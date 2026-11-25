#include "booksim.hpp"
#include "hier_chiplet.hpp"

#include <cassert>
#include <sstream>

using namespace std;

int gHierChiplets = 0;
int gHierIntNodes = 0;
int gHierChipletRouters = 0;
int gHierInterposerRouters = 0;

namespace {

int _interposer_base() { return gHierChiplets * gHierChipletRouters; }

void _VcRange(const Flit *f, int *begin, int *end)
{
  *begin = 0;
  *end = gNumVCs - 1;
  if (f->type == Flit::READ_REQUEST) {
    *begin = gReadReqBeginVC;
    *end = gReadReqEndVC;
  } else if (f->type == Flit::WRITE_REQUEST) {
    *begin = gWriteReqBeginVC;
    *end = gWriteReqEndVC;
  } else if (f->type == Flit::READ_REPLY) {
    *begin = gReadReplyBeginVC;
    *end = gReadReplyEndVC;
  } else if (f->type == Flit::WRITE_REPLY) {
    *begin = gWriteReplyBeginVC;
    *end = gWriteReplyEndVC;
  }
}

void _Add(OutputSet *outputs, int port, int begin, int end)
{
  outputs->Clear();
  outputs->AddRange(port, begin, end);
}

int _CurrentChiplet(const Router *r)
{
  return r->GetID() / gHierChipletRouters;
}

bool _IsInternalRouter(const Router *r)
{
  return r->GetID() < _interposer_base();
}

bool _IsGateway(const Router *r)
{
  return _IsInternalRouter(r) && (r->GetID() % gHierChipletRouters == 1);
}

int _RingTarget(const Flit *f, int source_chiplet)
{
  if (f->intm == 0) {
    return (source_chiplet + 1) % gHierInterposerRouters;
  }
  return (f->dest / gHierIntNodes) % gHierInterposerRouters;
}

void _RouteInterposer(const Router *r, const Flit *f, int begin, int end,
                      OutputSet *outputs, int target)
{
  const int current = r->GetID() - _interposer_base();
  if (current == target) {
    _Add(outputs, 0, begin, end); // bridge to the chiplet gateway
    return;
  }

  const bool wraps = target < current;
  if (wraps && current == gHierInterposerRouters - 1) {
    // The dateline transition moves to the upper half of the VC range.
    assert(begin + 1 < gNumVCs);
    f->ph = 1;
    _Add(outputs, 1, begin + 1, begin + 1);
  } else {
    _Add(outputs, 1, begin, begin);
  }
}

void _RouteHierarchical(const Router *r, const Flit *f, int in_channel,
                        OutputSet *outputs, bool inject, bool composite)
{
  int begin, end;
  _VcRange(f, &begin, &end);
  if (!inject && f->vc >= 0) {
    begin = f->vc;
    end = f->vc;
  }

  if (inject) {
    f->ph = 0;
    if (composite) {
      f->intm = 0;
    }
    // Start every packet in the lower dateline VC. The upper VC is entered
    // only when a clockwise route crosses the ring boundary.
    _Add(outputs, -1, begin, begin);
    return;
  }

  if (_IsInternalRouter(r)) {
    const int chiplet = _CurrentChiplet(r);
    const bool gateway = _IsGateway(r);

    if (!gateway) {
      if (composite && f->intm == 3) {
        _Add(outputs, f->dest % gHierIntNodes, begin, end);
      } else if (composite && f->intm == 1) {
        f->intm = 2;
        _Add(outputs, gHierIntNodes, begin, end);
      } else if (!composite && HierChiplet::ChipletOfNode(f->dest) == chiplet) {
        _Add(outputs, f->dest % gHierIntNodes, begin, end);
      } else {
        _Add(outputs, gHierIntNodes, begin, end);
      }
      return;
    }

    // Gateway port 0 is the internal link, port 1 is the interposer bridge.
    if (in_channel == 1) {
      if (composite) {
        if (f->intm == 0) {
          f->intm = 1;
        } else if (f->intm == 2) {
          f->intm = 3;
        }
      }
      _Add(outputs, 0, begin, end);
    } else {
      if (composite) {
        f->ph = 0;
      }
      _Add(outputs, 1, begin, end);
    }
    return;
  }

  const int source_chiplet = f->src / gHierIntNodes;
  const int target = composite
      ? _RingTarget(f, source_chiplet)
      : (f->dest / gHierIntNodes) % gHierInterposerRouters;
  _RouteInterposer(r, f, begin, end, outputs, target);
}

}

HierChiplet::HierChiplet(const Configuration &config, const string &name)
  : Network(config, name), _chiplets(0), _int_nodes(0),
    _chiplet_routers(0), _interposer_routers(0), _interposer_latency(0)
{
  _ComputeSize(config);
  _Alloc();
  _BuildNet(config);
}

HierChiplet::~HierChiplet()
{
}

void HierChiplet::_ComputeSize(const Configuration &config)
{
  _chiplets = config.GetInt("chiplet");
  _int_nodes = config.GetInt("int_nodes");
  _chiplet_routers = config.GetInt("chiplet_routers");
  _interposer_routers = config.GetInt("interposer_routers");
  _interposer_latency = config.GetInt("interposer_latency");

  if (_interposer_routers == 0) {
    _interposer_routers = _chiplets;
  }
  assert(_chiplets > 1);
  assert(_int_nodes > 0);
  assert(_chiplet_routers == 2);
  assert(_interposer_routers == _chiplets);
  assert(_interposer_latency > 0);

  _size = _chiplets * _chiplet_routers + _interposer_routers;
  _nodes = _chiplets * _int_nodes;
  _channels = 2 * _chiplets + 2 * _chiplets + _interposer_routers;

  gHierChiplets = _chiplets;
  gHierIntNodes = _int_nodes;
  gHierChipletRouters = _chiplet_routers;
  gHierInterposerRouters = _interposer_routers;
}

void HierChiplet::_BuildNet(const Configuration &config)
{
  for (int c = 0; c < _chiplets; ++c) {
    const int local = ChipletRouter(c, 0);
    const int gateway = ChipletRouter(c, 1);
    const int local_degree = _int_nodes + 1;

    ostringstream local_name;
    local_name << "chiplet_" << c << "_router_0";
    _routers[local] = Router::NewRouter(config, this, local_name.str(),
                                        local, local_degree, local_degree);
    _timed_modules.push_back(_routers[local]);

    ostringstream gateway_name;
    gateway_name << "chiplet_" << c << "_gateway";
    _routers[gateway] = Router::NewRouter(config, this, gateway_name.str(),
                                          gateway, 2, 2);
    _timed_modules.push_back(_routers[gateway]);

    for (int n = 0; n < _int_nodes; ++n) {
      const int node = c * _int_nodes + n;
      _routers[local]->AddInputChannel(_inject[node], _inject_cred[node]);
      _routers[local]->AddOutputChannel(_eject[node], _eject_cred[node]);
      _inject[node]->SetLatency(1);
      _eject[node]->SetLatency(1);
    }
  }

  for (int i = 0; i < _interposer_routers; ++i) {
    const int id = InterposerRouter(i);
    ostringstream name;
    name << "interposer_router_" << i;
    _routers[id] = Router::NewRouter(config, this, name.str(), id, 2, 2);
    _timed_modules.push_back(_routers[id]);
  }

  int ch = 0;
  for (int c = 0; c < _chiplets; ++c) {
    const int local = ChipletRouter(c, 0);
    const int gateway = ChipletRouter(c, 1);

    _chan[ch]->SetLatency(1);
    _chan_cred[ch]->SetLatency(1);
    _routers[local]->AddOutputChannel(_chan[ch], _chan_cred[ch]);
    _routers[gateway]->AddInputChannel(_chan[ch], _chan_cred[ch]);
    ++ch;

    _chan[ch]->SetLatency(1);
    _chan_cred[ch]->SetLatency(1);
    _routers[gateway]->AddOutputChannel(_chan[ch], _chan_cred[ch]);
    _routers[local]->AddInputChannel(_chan[ch], _chan_cred[ch]);
    ++ch;

    const int interposer = InterposerRouter(c);
    _chan[ch]->SetLatency(_interposer_latency);
    _chan_cred[ch]->SetLatency(_interposer_latency);
    _routers[gateway]->AddOutputChannel(_chan[ch], _chan_cred[ch]);
    _routers[interposer]->AddInputChannel(_chan[ch], _chan_cred[ch]);
    ++ch;

    _chan[ch]->SetLatency(_interposer_latency);
    _chan_cred[ch]->SetLatency(_interposer_latency);
    _routers[interposer]->AddOutputChannel(_chan[ch], _chan_cred[ch]);
    _routers[gateway]->AddInputChannel(_chan[ch], _chan_cred[ch]);
    ++ch;
  }

  for (int i = 0; i < _interposer_routers; ++i) {
    const int next = (i + 1) % _interposer_routers;
    _chan[ch]->SetLatency(_interposer_latency);
    _chan_cred[ch]->SetLatency(_interposer_latency);
    _routers[InterposerRouter(i)]->AddOutputChannel(_chan[ch], _chan_cred[ch]);
    _routers[InterposerRouter(next)]->AddInputChannel(_chan[ch], _chan_cred[ch]);
    ++ch;
  }

  assert(ch == _channels);
}

int HierChiplet::ChipletOfNode(int node)
{
  return node / gHierIntNodes;
}

int HierChiplet::LocalPortOfNode(int node)
{
  return node % gHierIntNodes;
}

int HierChiplet::ChipletRouter(int chiplet, int local_router)
{
  return chiplet * gHierChipletRouters + local_router;
}

int HierChiplet::InterposerRouter(int index)
{
  return _interposer_base() + index;
}

void HierChiplet::RegisterRoutingFunctions()
{
  gRoutingFunctionMap["dor_hier_chiplet"] = &dor_hier_chiplet;
  gRoutingFunctionMap["cycle_hier_chiplet"] = &cycle_hier_chiplet;
}

void dor_hier_chiplet(const Router *r, const Flit *f, int in_channel,
                     OutputSet *outputs, bool inject)
{
  _RouteHierarchical(r, f, in_channel, outputs, inject, false);
}

void cycle_hier_chiplet(const Router *r, const Flit *f, int in_channel,
                       OutputSet *outputs, bool inject)
{
  _RouteHierarchical(r, f, in_channel, outputs, inject, true);
}
