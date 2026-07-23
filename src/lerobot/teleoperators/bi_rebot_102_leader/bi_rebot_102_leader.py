#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from functools import cached_property
from typing import Any

from lerobot.types import RobotAction
from lerobot.utils.bimanual import BimanualMixin
from lerobot.utils.decorators import check_if_not_connected

from ..rebot_102_leader import RebotArm102Leader, RebotArm102LeaderTeleopConfig
from ..teleoperator import Teleoperator
from .config_bi_rebot_102_leader import BiRebot102LeaderConfig

logger = logging.getLogger(__name__)


class BiRebot102Leader(BimanualMixin, Teleoperator):
    """Bimanual Seeed Studio StarArm102 / reBot Arm 102 leader.

    Composes two single-arm :class:`RebotArm102Leader` instances. Action keys of
    each arm are namespaced with a ``left_`` / ``right_`` prefix, so a bimanual
    leader can teleoperate a bimanual reBot B601 follower.
    """

    config_class = BiRebot102LeaderConfig
    name = "bi_rebot_102_leader"

    def __init__(self, config: BiRebot102LeaderConfig):
        super().__init__(config)
        self.config = config

        left_arm_config = RebotArm102LeaderTeleopConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            baudrate=config.left_arm_config.baudrate,
            joint_ids=config.left_arm_config.joint_ids,
            joint_directions=config.left_arm_config.joint_directions,
            joint_ranges=config.left_arm_config.joint_ranges,
        )

        right_arm_config = RebotArm102LeaderTeleopConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            baudrate=config.right_arm_config.baudrate,
            joint_ids=config.right_arm_config.joint_ids,
            joint_directions=config.right_arm_config.joint_directions,
            joint_ranges=config.right_arm_config.joint_ranges,
        )

        self.left_arm = RebotArm102Leader(left_arm_config)
        self.right_arm = RebotArm102Leader(right_arm_config)

        # The two leaders sit on physically independent serial buses (separate
        # FashionStarServo instances), so their blocking reads run truly concurrently
        # (the GIL is released during the serial waits). A persistent 2-worker pool
        # avoids per-tick thread creation. Shut down in disconnect().
        self._io_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix='bi_rebot_io')

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **{f"left_{k}": v for k, v in self.left_arm.action_features.items()},
            **{f"right_{k}": v for k, v in self.right_arm.action_features.items()},
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    def _run_both(self, left_fn: Callable[[], Any], right_fn: Callable[[], Any]) -> tuple[Any, Any]:
        """Run the left and right leader reads concurrently and return (left, right).

        Each leader owns a separate serial bus and FashionStarServo instance, so there
        is no shared mutable state between the two callables. Running them on the pool
        overlaps the two buses' round-trips, roughly halving per-tick read wall time.
        Exceptions from either arm propagate via future.result().
        """
        left_future = self._io_pool.submit(left_fn)
        right_future = self._io_pool.submit(right_fn)
        # Always wait on both futures so no arm's read is left dangling, even if one raises.
        left_result = left_future.result()
        right_result = right_future.result()
        return left_result, right_result

    @check_if_not_connected
    def disconnect(self) -> None:
        # Stop the I/O pool first so no worker touches a bus mid-disconnect, then let
        # BimanualMixin.disconnect() close both arms.
        self._io_pool.shutdown(wait=True)
        super().disconnect()

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        left_action, right_action = self._run_both(self.left_arm.get_action, self.right_arm.get_action)
        action_dict = {}
        action_dict.update({f"left_{k}": v for k, v in left_action.items()})
        action_dict.update({f"right_{k}": v for k, v in right_action.items()})
        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError("Feedback is not implemented for the reBot Arm 102 leader.")
