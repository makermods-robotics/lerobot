// Vendor-exact KDL oracle: reproduces kdl_solver.cpp's dynamics path
// (kdl_parser::treeFromFile -> getChain("base_link","Link6") -> ChainDynParam).
// Reads rows of 12 doubles from stdin: q[6] qd[6].
// Writes CSV rows of 12: gravity[6], coriolis[6].
// Build (needs ROS Humble + orocos-kdl); see README.md.
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <cstdio>
int main(int argc, char** argv){
  KDL::Tree tree;
  if(!kdl_parser::treeFromFile(argv[1], tree)){ fprintf(stderr,"tree fail\n"); return 1; }
  KDL::Chain chain;
  if(!tree.getChain("base_link","Link6",chain)){ fprintf(stderr,"chain fail\n"); return 1; }
  KDL::ChainDynParam dyn(chain, KDL::Vector(0,0,-9.81));
  int n=chain.getNrOfJoints();
  KDL::JntArray q(n), qd(n), g(n), c(n);
  double v;
  while(true){
    for(int i=0;i<n;i++){ if(scanf("%lf",&v)!=1) return 0; q(i)=v; }
    for(int i=0;i<n;i++){ if(scanf("%lf",&v)!=1) return 0; qd(i)=v; }
    dyn.JntToGravity(q,g);
    dyn.JntToCoriolis(q,qd,c);
    for(int i=0;i<n;i++) printf("%.12g,", g(i));
    for(int i=0;i<n;i++) printf("%.12g%c", c(i), i==n-1?'\n':',');
    fflush(stdout);
  }
}
