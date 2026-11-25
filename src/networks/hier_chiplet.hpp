#ifndef _HIER_CHIPLET_HPP_
#define _HIER_CHIPLET_HPP_

#include "network.hpp"
#include "routefunc.hpp"

class HierChiplet : public Network {
  int _chiplets;
  int _int_nodes;
  int _chiplet_routers;
  int _interposer_routers;
  int _interposer_latency;

  void _ComputeSize(const Configuration &config);
  void _BuildNet(const Configuration &config);

public:
  HierChiplet(const Configuration &config, const string &name);
  ~HierChiplet();

  static void RegisterRoutingFunctions();
  static int ChipletOfNode(int node);
  static int LocalPortOfNode(int node);
  static int ChipletRouter(int chiplet, int local_router);
  static int InterposerRouter(int index);
};

void dor_hier_chiplet(const Router *r, const Flit *f, int in_channel,
                      OutputSet *outputs, bool inject);

void cycle_hier_chiplet(const Router *r, const Flit *f, int in_channel,
                        OutputSet *outputs, bool inject);

extern int gHierChiplets;
extern int gHierIntNodes;
extern int gHierChipletRouters;
extern int gHierInterposerRouters;

#endif
