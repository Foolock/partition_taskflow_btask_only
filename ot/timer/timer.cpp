#include <ot/timer/timer.hpp>
#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

namespace ot {

//
//Pin pin;
//PIN_STATUS stat;
//
//stat == PIN_STATUS::NON;
//
//stat == 0
//
// ------------------------------------------------------------------------------------------------

// Function: set_num_threads
Timer& Timer::set_num_threads(unsigned n) {
  std::scoped_lock lock(_mutex);
  unsigned w = (n == 0) ? 0 : n-1;
  OT_LOGI("using ", n, " threads (", w, " worker)");
  // TODO
  //_taskflow.num_workers(w);
  return *this;
}

// Procedure: _add_to_lineage
void Timer::_add_to_lineage(tf::Task task) {
  _lineage | [&] (auto& p) { p.precede(task); };
  _lineage = task;
}

// Function: _max_pin_name_size
size_t Timer::_max_pin_name_size() const {
  if(_pins.empty()) {
    return 0;
  }
  else {
    return std::max_element(_pins.begin(), _pins.end(), 
      [] (const auto& l, const auto& r) {
        return l.second._name.size() < r.second._name.size();
      }
    )->second._name.size();
  }
}

// Function: _max_net_name_size
size_t Timer::_max_net_name_size() const {
  if(_nets.empty()) {
    return 0;
  }
  else {
    return std::max_element(_nets.begin(), _nets.end(), 
      [] (const auto& l, const auto& r) {
        return l.second._name.size() < r.second._name.size();
      }
    )->second._name.size();
  }
}

// Function: repower_gate
// Change the size or level of an existing gate, e.g., NAND2_X2 to NAND2_X3. The gate's
// logic function and topology is guaranteed to be the same, along with the currently-connected
// nets. However, the pin capacitances of the new cell type might be different. 
Timer& Timer::repower_gate(std::string gate, std::string cell) {

  std::scoped_lock lock(_mutex);

  auto task = _taskflow.emplace([this, gate=std::move(gate), cell=std::move(cell)] () {
    _repower_gate(gate, cell);
  });
  
  _add_to_lineage(task);

  return *this;
}

// Procedure: _repower_gate
void Timer::_repower_gate(const std::string& gname, const std::string& cname) {
  
  OT_LOGE_RIF(!_celllib[MIN] || !_celllib[MAX], "celllib not found");

  // Insert the gate if it doesn't exist.
  if(auto gitr = _gates.find(gname); gitr == _gates.end()) {
    OT_LOGW("gate ", gname, " doesn't exist (insert instead)");
    _insert_gate(gname, cname);
    return;
  }
  else {

    auto cell = CellView {_celllib[MIN]->cell(cname), _celllib[MAX]->cell(cname)};

    OT_LOGE_RIF(!cell[MIN] || !cell[MAX], "cell ", cname, " not found");

    auto& gate = gitr->second;

    // Remap the cellpin
    for(auto pin : gate._pins) {
      FOR_EACH_EL(el) {
        assert(pin->cellpin(el));
        if(const auto cpin = cell[el]->cellpin(pin->cellpin(el)->name)) {
          pin->_remap_cellpin(el, *cpin);
        }
        else {
          OT_LOGE(
            "repower ", gname, " with ", cname, " failed (cellpin mismatched)"
          );  
        }
      }
    }
    
    gate._cell = cell;

    // reconstruct the timing and tests
    _remove_gate_arcs(gate);
    _insert_gate_arcs(gate);

    // Insert the gate to the frontier
    for(auto pin : gate._pins) {
      _insert_frontier(*pin);
      for(auto arc : pin->_fanin) {
        _insert_frontier(arc->_from);
      }
    }
  }
}

// Fucntion: insert_gate
// Create a new gate in the design. This newly-created gate is "not yet" connected to
// any other gates or wires. The gate to insert cannot conflict with existing gates.
Timer& Timer::insert_gate(std::string gate, std::string cell) {  
  
  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, gate=std::move(gate), cell=std::move(cell)] () {
    _insert_gate(gate, cell);
  });

  _add_to_lineage(op);

  return *this;
}

// Function: _insert_gate
void Timer::_insert_gate(const std::string& gname, const std::string& cname) {

  OT_LOGE_RIF(!_celllib[MIN] || !_celllib[MAX], "celllib not found");

  if(_gates.find(gname) != _gates.end()) {
    OT_LOGW("gate ", gname, " already existed");
    return;
  }

  auto cell = CellView {_celllib[MIN]->cell(cname), _celllib[MAX]->cell(cname)};

  if(!cell[MIN] || !cell[MAX]) {
    OT_LOGE("cell ", cname, " not found in celllib");
    return;
  }
  
  auto& gate = _gates.try_emplace(gname, gname, cell).first->second;
  
  // Insert pins
  for(const auto& [cpname, ecpin] : cell[MIN]->cellpins) {

    CellpinView cpv {&ecpin, cell[MAX]->cellpin(cpname)};

    if(!cpv[MIN] || !cpv[MAX]) {
      OT_LOGF("cellpin ", cpname, " mismatched in celllib");
    }

    auto& pin = _insert_pin(gname + ':' + cpname);
    pin._handle = cpv;
    pin._gate = &gate;
    
    gate._pins.push_back(&pin);
  }
  
  _insert_gate_arcs(gate);
}

// Fucntion: remove_gate
// Remove a gate from the current design. This is guaranteed to be called after the gate has 
// been disconnected from the design using pin-level operations. The procedure iterates all 
// pins in the cell to which the gate was attached. Each pin that is being iterated is either
// a cell input pin or cell output pin. In the former case, the pin might have constraint arc
// while in the later case, the ot_pin.has no output connections and all fanin edges should be 
// removed here.
Timer& Timer::remove_gate(std::string gate) {  
  
  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, gate=std::move(gate)] () {
    if(auto gitr = _gates.find(gate); gitr != _gates.end()) {
      _remove_gate(gitr->second);
    }
  });

  _add_to_lineage(op);

  return *this;
}

// Procedure: _remove_gate
void Timer::_remove_gate(Gate& gate) {

  // Disconnect this gate from the design.
  for(auto pin : gate._pins) {
    _disconnect_pin(*pin);
  }

  // Remove associated test
  for(auto test : gate._tests) {
    _remove_test(*test);
  }

  // Remove associated arcs
  for(auto arc : gate._arcs) {
    _remove_arc(*arc);
  }

  // Disconnect the gate and remove the pins from the gate
  for(auto pin : gate._pins) {
    _remove_pin(*pin);
  }

  // remove the gate
  _gates.erase(gate._name);
}

// Procedure: _remove_gate_arcs
void Timer::_remove_gate_arcs(Gate& gate) {

  // remove associated tests
  for(auto test : gate._tests) {
    _remove_test(*test);
  }
  gate._tests.clear();
  
  // remove associated arcs
  for(auto arc : gate._arcs) {
    _remove_arc(*arc);
  }
  gate._arcs.clear();
}

// Procedure: _insert_gate_arcs
void Timer::_insert_gate_arcs(Gate& gate) {

  assert(gate._tests.empty() && gate._arcs.empty());

  FOR_EACH_EL(el) {
    for(const auto& [cpname, cp] : gate._cell[el]->cellpins) {
      auto& to_pin = _insert_pin(gate._name + ':' + cpname);

      for(const auto& tm : cp.timings) {

        if(_is_redundant_timing(tm, el)) {
          continue;
        }

        TimingView tv{nullptr, nullptr};
        tv[el] = &tm;

        auto& from_pin = _insert_pin(gate._name + ':' + tm.related_pin);
        auto& arc = _insert_arc(from_pin, to_pin, tv);
        
        gate._arcs.push_back(&arc);
        if(tm.is_constraint()) {
          auto& test = _insert_test(arc);
          gate._tests.push_back(&test);
        }
      }
    }
  }
}

// Function: connect_pin
// Connect the pin to the corresponding net. The pin_name will either have the 
// <gate name>:<cell pin name> syntax (e.g., u4:ZN) or be a primary input. The net name
// will match an existing net read in from a .spef file.
Timer& Timer::connect_pin(std::string pin, std::string net) {

  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, pin=std::move(pin), net=std::move(net)] () {
    auto p = _pins.find(pin);
    auto n = _nets.find(net);
    OT_LOGE_RIF(p==_pins.end() || n == _nets.end(),
      "can't connect pin ", pin,  " to net ", net, " (pin/net not found)"
    )
    _connect_pin(p->second, n->second);
  });

  _add_to_lineage(op);

  return *this;
}

// Procedure: _connect_pin
void Timer::_connect_pin(Pin& pin, Net& net) {
      
  // Connect the pin to the net and construct the edge connections.
  net._insert_pin(pin);
  
  // Case 1: the pin is the root of the net.
  if(&pin == net._root) {
    for(auto leaf : net._pins) {
      if(leaf != &pin) {
        _insert_arc(pin, *leaf, net);
      }
    }
  }
  // Case 2: the pin is not a root of the net.
  else {
    if(net._root) {
      _insert_arc(*net._root, pin, net);
    }
  }

  // TODO(twhuang) Enable the clock tree update?
}

// Procedure: disconnect_pin
// Disconnect the pin from the net it is connected to. The pin_name will either have the 
// <gate name>:<cell pin name> syntax (e.g., u4:ZN) or be a primary input.
Timer& Timer::disconnect_pin(std::string name) {
  
  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, name=std::move(name)] () {
    if(auto itr = _pins.find(name); itr != _pins.end()) {
      _disconnect_pin(itr->second);
    }
  });

  _add_to_lineage(op);

  return *this;
}

// Procedure: disconnect_pin
// TODO (twhuang)
// try get rid of find_fanin which can be wrong under multiple arcs.
void Timer::_disconnect_pin(Pin& pin) {

  auto net = pin._net;

  if(net == nullptr) return;

  // Case 1: the pin is a root of the net (i.e., root of the rctree)
  if(&pin == net->_root) {
    // Iterate the pinlist and delete the corresponding edge. Notice here we cannot iterate
    // fanout of the node during removal.
    for(auto leaf : net->_pins) {
      if(leaf != net->_root) {
        auto arc = leaf->_find_fanin(*net->_root);
        assert(arc);
        _remove_arc(*arc);
      }
    }
  }
  // Case 2: the pin is not a root of the net.
  else {
    if(net->_root) {
      auto arc = pin._find_fanin(*net->_root);
      assert(arc);
      _remove_arc(*arc);
    }
  }
  
  // TODO: Enable the clock tree update.
  
  // Remove the pin from the net and enable the rc timing update.
  net->_remove_pin(pin);
}

// Function: insert_net
// Creates an empty net object with the input "net_name". By default, it will not be connected 
// to any pins and have no parasitics (.spef). This net will be connected to existing pins in 
// the design by the "connect_pin" and parasitics will be loaded by "spef".
Timer& Timer::insert_net(std::string name) {

  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, name=std::move(name)] () {
    _insert_net(name);
  });

  _add_to_lineage(op);

  return *this;
}

// Function: _insert_net
Net& Timer::_insert_net(const std::string& name) {
  return _nets.try_emplace(name, name).first->second;
}

// Procedure: remove_net
// Remove a net from the current design, which by default removes all associated pins.
Timer& Timer::remove_net(std::string name) {

  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, name=std::move(name)] () {
    if(auto itr = _nets.find(name); itr != _nets.end()) {
      _remove_net(itr->second);
    }
  });

  _add_to_lineage(op);

  return *this;
}

// Function: _remove_net
void Timer::_remove_net(Net& net) {

  if(net.num_pins() > 0) {
    auto fetch = net._pins;
    for(auto pin : fetch) {
      _disconnect_pin(*pin);
    }
  }

  _nets.erase(net._name);
}

// Function: _insert_pin
Pin& Timer::_insert_pin(const std::string& name) {
  
  // pin already exists
  if(auto [itr, inserted] = _pins.try_emplace(name, name); !inserted) {
    return itr->second;
  }
  // inserted a new pon
  else {
    
    // Generate the pin idx
    auto& pin = itr->second;
    
    // Assign the idx mapping
    pin._idx = _pin_idx_gen.get();
    resize_to_fit(pin._idx + 1, _idx2pin);
    _idx2pin[pin._idx] = &pin;

    // insert to frontier
    _insert_frontier(pin);

    return pin;
  }
}

// Function: _remove_pin
void Timer::_remove_pin(Pin& pin) {

  assert(pin.num_fanouts() == 0 && pin.num_fanins() == 0 && pin.net() == nullptr);

  _remove_frontier(pin);

  // remove the id mapping
  _idx2pin[pin._idx] = nullptr;
  _pin_idx_gen.recycle(pin._idx);

  // remove the pin
  _pins.erase(pin._name);
}

// Function: cppr
Timer& Timer::cppr(bool flag) {
  
  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, flag] () {
    _cppr(flag);
  });

  _add_to_lineage(op);

  return *this;
}

// Procedure: _cppr
// Enable/Disable common path pessimism removal (cppr) analysis
void Timer::_cppr(bool enable) {
  
  // nothing to do.
  if((enable && _cppr_analysis) || (!enable && !_cppr_analysis)) {
    return;
  }

  if(enable) {
    OT_LOGI("enable cppr analysis");
    _cppr_analysis.emplace();
  }
  else {
    OT_LOGI("disable cppr analysis");
    _cppr_analysis.reset();
  }
    
  for(auto& test : _tests) {
    _insert_frontier(test._constrained_pin());
  }
}

// Function: clock
Timer& Timer::create_clock(std::string c, std::string s, float p) {
  
  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, c=std::move(c), s=std::move(s), p] () {
    if(auto itr = _pins.find(s); itr != _pins.end()) {
      _create_clock(c, itr->second, p);
    }
    else {
      OT_LOGE("can't create clock ", c, " on source ", s, " (pin not found)");
    }
  });

  _add_to_lineage(op);
  
  return *this;
}

// Function: create_clock
Timer& Timer::create_clock(std::string c, float p) {
  
  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, c=std::move(c), p] () {
    _create_clock(c, p);
  });

  _add_to_lineage(op);

  return *this;
}

// Procedure: _create_clock
Clock& Timer::_create_clock(const std::string& name, Pin& pin, float period) {
  auto& clock = _clocks.try_emplace(name, name, pin, period).first->second;
  _insert_frontier(pin);
  return clock;
}

// Procedure: _create_clock
Clock& Timer::_create_clock(const std::string& name, float period) {
  auto& clock = _clocks.try_emplace(name, name, period).first->second;
  return clock;
}

// Function: insert_primary_input
Timer& Timer::insert_primary_input(std::string name) {

  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, name=std::move(name)] () {
    _insert_primary_input(name);
  });

  _add_to_lineage(op);

  return *this;
}

// Procedure: _insert_primary_input
void Timer::_insert_primary_input(const std::string& name) {

  if(_pis.find(name) != _pis.end()) {
    OT_LOGW("can't insert PI ", name, " (already existed)");
    return;
  }

  assert(_pins.find(name) == _pins.end());

  // Insert the pin and and pi
  auto& pin = _insert_pin(name);
  auto& pi = _pis.try_emplace(name, pin).first->second;
  
  // Associate the connection.
  pin._handle = &pi;

  // Insert the pin to the frontier
  _insert_frontier(pin);

  // Create a net for the po and connect the pin to the net.
  auto& net = _insert_net(name); 
  
  // Connect the pin to the net.
  _connect_pin(pin, net);
}

// Function: insert_primary_output
Timer& Timer::insert_primary_output(std::string name) {

  std::scoped_lock lock(_mutex);

  auto op = _taskflow.emplace([this, name=std::move(name)] () {
    _insert_primary_output(name);
  });

  _add_to_lineage(op);

  return *this;
}

// Procedure: _insert_primary_output
void Timer::_insert_primary_output(const std::string& name) {

  if(_pos.find(name) != _pos.end()) {
    OT_LOGW("can't insert PO ", name, " (already existed)");
    return;
  }

  assert(_pins.find(name) == _pins.end());

  // Insert the pin and and pi
  auto& pin = _insert_pin(name);
  auto& po = _pos.try_emplace(name, pin).first->second;
  
  // Associate the connection.
  pin._handle = &po;

  // Insert the pin to the frontier
  _insert_frontier(pin);

  // Create a net for the po and connect the pin to the net.
  auto& net = _insert_net(name); 

  // Connect the pin to the net.
  _connect_pin(pin, net);
}

// Procedure: _insert_test
Test& Timer::_insert_test(Arc& arc) {
  auto& test = _tests.emplace_front(arc);
  test._satellite = _tests.begin();
  test._pin_satellite = arc._to._tests.insert(arc._to._tests.end(), &test);
  return test;
}

// Procedure: _remove_test
void Timer::_remove_test(Test& test) {
  assert(test._satellite);
  if(test._pin_satellite) {
    test._arc._to._tests.erase(*test._pin_satellite);
  }
  _tests.erase(*test._satellite);
}

// Procedure: _remove_arc
// Remove an arc from the design. The procedure first disconnects the arc from its two ending
// pins, "from_pin" and "to_pin". Then it removes the arc from the design and insert both
// "from_pin" and "to_pin" into the pipeline.
void Timer::_remove_arc(Arc& arc) {

  assert(arc._satellite);
  
  arc._from._remove_fanout(arc);
  arc._to._remove_fanin(arc);

  // Insert the two ends to the frontier list.
  _insert_frontier(arc._from, arc._to);
  
  // remove the id mapping
  _idx2arc[arc._idx] = nullptr;
  _arc_idx_gen.recycle(arc._idx);

  // Remove this arc from the timer.
  _arcs.erase(*arc._satellite);
}

// Function: _insert_arc (net arc)
// Insert an net arc to the timer.
Arc& Timer::_insert_arc(Pin& from, Pin& to, Net& net) {

  OT_LOGF_IF(&from == &to, "net arc is a self loop at ", to._name);

  // Create a new arc
  auto& arc = _arcs.emplace_front(from, to, net);
  arc._satellite = _arcs.begin();

  from._insert_fanout(arc);
  to._insert_fanin(arc);

  // Insert frontiers
  _insert_frontier(from, to);
   
  // Assign the idx mapping
  arc._idx = _arc_idx_gen.get();
  resize_to_fit(arc._idx + 1, _idx2arc);
  _idx2arc[arc._idx] = &arc;

  return arc;
}

// Function: _insert_arc (cell arc)
// Insert a cell arc to the timing graph. A cell arc is a combinational link.
Arc& Timer::_insert_arc(Pin& from, Pin& to, TimingView tv) {
  
  //OT_LOGF_IF(&from == &to, "timing graph contains a self loop at ", to._name);

  // Create a new arc
  auto& arc = _arcs.emplace_front(from, to, tv);
  arc._satellite = _arcs.begin();
  from._insert_fanout(arc);
  to._insert_fanin(arc);

  // insert the arc into frontier list.
  _insert_frontier(from, to);
  
  // Assign the idx mapping
  arc._idx = _arc_idx_gen.get();
  resize_to_fit(arc._idx + 1, _idx2arc);
  _idx2arc[arc._idx] = &arc;

  return arc;
}

// Procedure: _fprop_rc_timing
void Timer::_fprop_rc_timing(Pin& pin) {
  if(auto net = pin._net; net) {
    net->_update_rc_timing();
  }
}

// Procedure: _fprop_slew
void Timer::_fprop_slew(Pin& pin) {
  
  // clear slew  
  pin._reset_slew();

  // PI
  if(auto pi = pin.primary_input(); pi) {
    FOR_EACH_EL_RF_IF(el, rf, pi->_slew[el][rf]) {
      pin._relax_slew(nullptr, el, rf, el, rf, *(pi->_slew[el][rf]));
    }
  }
  
  // Relax the slew from its fanin.
  for(auto arc : pin._fanin) {
    arc->_fprop_slew();
  }
}

// Procedure: _fprop_delay
void Timer::_fprop_delay(Pin& pin) {

  // clear delay
  for(auto arc : pin._fanin) {
    arc->_reset_delay();
  }

  // Compute the delay from its fanin.
  for(auto arc : pin._fanin) {
    arc->_fprop_delay();
  }
}

// Procedure: _fprop_at
void Timer::_fprop_at(Pin& pin) {
  
  // clear at
  pin._reset_at();

  // PI
  if(auto pi = pin.primary_input(); pi) {
    FOR_EACH_EL_RF_IF(el, rf, pi->_at[el][rf]) {
      pin._relax_at(nullptr, el, rf, el, rf, *(pi->_at[el][rf]));
    }
  }

  // Relax the at from its fanin.
  for(auto arc : pin._fanin) {
    arc->_fprop_at();
  }
}

// Procedure: _fprop_test
void Timer::_fprop_test(Pin& pin) {
  
  // reset tests
  for(auto test : pin._tests) {
    test->_reset();
  }
  
  // Obtain the rat
  if(!_clocks.empty()) {

    // Update the rat
    for(auto test : pin._tests) {
      // TODO: currently we assume a single clock...
      test->_fprop_rat(_clocks.begin()->second._period);
      
      // compute the cppr credit if any
      if(_cppr_analysis) {
        FOR_EACH_EL_RF_IF(el, rf, test->raw_slack(el, rf)) {
          test->_cppr_credit[el][rf] = _cppr_credit(*test, el, rf);
        }
      }
    }
  }
}

// Procedure: _bprop_rat
void Timer::_bprop_rat(Pin& pin) {

  pin._reset_rat();

  // PO
  if(auto po = pin.primary_output(); po) {
    FOR_EACH_EL_RF_IF(el, rf, po->_rat[el][rf]) {
      pin._relax_rat(nullptr, el, rf, el, rf, *(po->_rat[el][rf]));
    }
  }

  // Test
  for(auto test : pin._tests) {
    FOR_EACH_EL_RF_IF(el, rf, test->_rat[el][rf]) {
      if(test->_cppr_credit[el][rf]) {
        pin._relax_rat(
          &test->_arc, el, rf, el, rf, *test->_rat[el][rf] + *test->_cppr_credit[el][rf]
        );
      }
      else {
        pin._relax_rat(&test->_arc, el, rf, el, rf, *test->_rat[el][rf]);
      }
    }
  }

  // Relax the rat from its fanout.
  for(auto arc : pin._fanout) {
    arc->_bprop_rat();
  }
}

// Procedure: _build_fprop_cands
// Performs DFS to find all nodes in the fanout cone of frontiers.
void Timer::_build_fprop_cands(Pin& from) {
  
  assert(!from._has_state(Pin::FPROP_CAND) && !from._has_state(Pin::IN_FPROP_STACK));

  from._insert_state(Pin::FPROP_CAND | Pin::IN_FPROP_STACK);

  for(auto arc : from._fanout) {
    if(auto& to = arc->_to; !to._has_state(Pin::FPROP_CAND)) {
      _build_fprop_cands(to);
    }
    else if(to._has_state(Pin::IN_FPROP_STACK)) {
      _scc_analysis = true;
    }
  }
  
  _fprop_cands.push_front(&from);  // insert from front for scc traversal
  from._remove_state(Pin::IN_FPROP_STACK);
}

// Procedure: _build_bprop_cands
// Perform the DFS to find all nodes in the fanin cone of fprop candidates.
void Timer::_build_bprop_cands(Pin& to) {
  
  assert(!to._has_state(Pin::BPROP_CAND) && !to._has_state(Pin::IN_BPROP_STACK));

  to._insert_state(Pin::BPROP_CAND | Pin::IN_BPROP_STACK);

  // add pin to scc
  if(_scc_analysis && to._has_state(Pin::FPROP_CAND) && !to._scc) {
    _scc_cands.push_back(&to);
  }

  for(auto arc : to._fanin) {
    if(auto& from=arc->_from; !from._has_state(Pin::BPROP_CAND)) {
      _build_bprop_cands(from);
    }
  }
  
  _bprop_cands.push_front(&to);
  to._remove_state(Pin::IN_BPROP_STACK);
}

// Procedure: _build_prop_cands
void Timer::_build_prop_cands() {

  _scc_analysis = false;

  // Discover all fprop candidates.
  for(const auto& ftr : _frontiers) {
    if(ftr->_has_state(Pin::FPROP_CAND)) {
      continue;
    }
    _build_fprop_cands(*ftr);
  }

  // Discover all bprop candidates.
  for(auto fcand : _fprop_cands) {

    if(fcand->_has_state(Pin::BPROP_CAND)) {
      continue;
    }

    _scc_cands.clear();
    _build_bprop_cands(*fcand);

    if(!_scc_analysis) {
      assert(_scc_cands.empty());
    }
    
    // here dfs returns with exacly one scc if exists
    if(auto& c = _scc_cands; c.size() >= 2 || (c.size() == 1 && c[0]->has_self_loop())) {
      auto& scc = _insert_scc(c);
      scc._unloop();
    }
  }
}

// Procedure: _build_prop_tasks
void Timer::_build_prop_tasks() {
  
  // explore propagation candidates, i.e., identifying affected region and add them to the candidate set(each entry is pin)
  _build_prop_cands();

  // Emplace the fprop task, this task is for each pin in the affected region
  // (1) propagate the rc timing
  // (2) propagate the slew 
  // (3) propagate the delay
  // (4) propagate the arrival time.
  for(auto pin : _fprop_cands) { // std::deque<Pin*> _fprop_cands;
    assert(!pin->_ftask);
    pin->_ftask = _taskflow.emplace([this, pin] () { // std::optional<tf::Task> _ftask; // std::optional 不用担心额外的动态内存分配 c++17
      _fprop_rc_timing(*pin); // 5 typical tasks for a pin to update timing in forward propagation // most time consuming if pin has net
      _fprop_slew(*pin);
      _fprop_delay(*pin);
      _fprop_at(*pin);
      _fprop_test(*pin);
    }).name(pin->_name);
  }
  
  // Build the dependency, this is the forward part of the dependency graph
  for(auto to : _fprop_cands) {
    for(auto arc : to->_fanin) { // std::list<RctEdge*> _fanin;
      if(arc->_has_state(Arc::LOOP_BREAKER)) {
        continue;
      }
      if(auto& from = arc->_from; from._has_state(Pin::FPROP_CAND)) {
        from._ftask->precede(to->_ftask.value());
      }
    }
  }

  // Emplace the bprop task
  // (1) propagate the required arrival time
  for(auto pin : _bprop_cands) {
    assert(!pin->_btask);
    pin->_btask = _taskflow.emplace([this, pin] () {
      _bprop_rat(*pin); // 1 typical task for a pin to update timing in backward progragation
    }).name(pin->_name);
  }

  // Build the task dependencies.
  for(auto to : _bprop_cands) {
    for(auto arc : to->_fanin) {
      if(arc->_has_state(Arc::LOOP_BREAKER)) {
        continue;
      }
      if(auto& from = arc->_from; from._has_state(Pin::BPROP_CAND)) {
        to->_btask->precede(from._btask.value());
      }
    } 
  }

  // Connect with ftasks
  for(auto pin : _bprop_cands) {
    if(pin->_btask->num_dependents() == 0 && pin->_ftask) {
      pin->_ftask->precede(pin->_btask.value()); // .value() check if the tf::Task exist, otherwise throw an exception 
    }
  }

}


// Procedure: _clear_prop_tasks
void Timer::_clear_prop_tasks() {
  
  // fprop is a subset of bprop
  for(auto pin : _bprop_cands) {
    pin->_ftask.reset();
    pin->_btask.reset();
    pin->_remove_state();
  }

  _fprop_cands.clear();
  _bprop_cands.clear();
}

// Function: update_timing
// Perform comprehensive timing update: 
// (1) grpah-based timing (GBA)
// (2) path-based timing (PBA)
void Timer::update_timing() {
  std::scoped_lock lock(_mutex);
  _update_timing();
}

// Function: _update_timing
void Timer::_update_timing() {

  // Timing is update-to-date
  if(!_lineage) {
    assert(_frontiers.size() == 0);
    return;
  }

  // materialize the lineage
  _executor.run(_taskflow).wait();
  _taskflow.clear();
  _lineage.reset();

  // Check if full update is required
//  if(_has_state(FULL_TIMING)) {
    _insert_full_timing_frontiers();
//  }

  // build propagation tasks
//  _build_prop_tasks();
  _build_prop_cands();

//  _taskflow.dump(std::cout);

  // cluster the task graph by adding cone_id to each pin and get cluster
  auto start = std::chrono::steady_clock::now();
  _cluster_graph();
  auto end = std::chrono::steady_clock::now();
  cluster_runtime += std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();

  start = std::chrono::steady_clock::now();
  // get partitions
  _get_partition();
  end = std::chrono::steady_clock::now();
  partition_runtime += std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();

  // get topologically sorted pins of each partition
  _get_topo_order();

  // clear original taskflow and build partition_taskflow
  _taskflow.clear();
  _build_partitioned_taskflow(); 
//  _execute_task_manually();

  // Execute the task
  start = std::chrono::steady_clock::now();
  _executor.run(_taskflow).wait();
  end = std::chrono::steady_clock::now();
  taskflow_runtime += std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
  _taskflow.clear();

//  _check_mtkahypar();

  // clear repcut used varibles
  _reset_repcut();
  
  // Clear the propagation tasks.
  _clear_prop_tasks();

  // Clear frontiers
  _clear_frontiers();

  // clear the state
  _remove_state();
}

// Procedure: _update_area
void Timer::_update_area() {
  
  _update_timing();

  if(_has_state(AREA_UPDATED)) {
    return;
  }
  
  _area = 0.0f;

  for(const auto& kvp : _gates) {
    if(const auto& c = kvp.second._cell[MIN]; c->area) {
      _area = *_area + *c->area;
    }
    else {
      OT_LOGE("cell ", c->name, " has no area defined");
      _area.reset();
      break;
    }
  }

  _insert_state(AREA_UPDATED);
}

// Procedure: _update_power
void Timer::_update_power() {

  _update_timing();

  if(_has_state(POWER_UPDATED)) {
    return;
  }

  // Update the static leakage power
  _leakage_power = 0.0f;
  
  for(const auto& kvp : _gates) {
    if(const auto& c = kvp.second._cell[MIN]; c->leakage_power) {
      _leakage_power = *_leakage_power + *c->leakage_power;
    }
    else {
      OT_LOGE("cell ", c->name, " has no leakage_power defined");
      _leakage_power.reset();
      break;
    }
  }

  _insert_state(POWER_UPDATED);
}

// Procedure: _update_endpoints
void Timer::_update_endpoints() {

  _update_timing();

  if(_has_state(EPTS_UPDATED)) {
    return;
  }

  // reset the storage and build task
  FOR_EACH_EL_RF(el, rf) {

    _endpoints[el][rf].clear();
    
    _taskflow.emplace([this, el=el, rf=rf] () {

      // for each po
      for(auto& po : _pos) {
        if(po.second.slack(el, rf).has_value()) {
          _endpoints[el][rf].emplace_back(el, rf, po.second);
        }
      }

      // for each test
      for(auto& test : _tests) {
        if(test.slack(el, rf).has_value()) {
          _endpoints[el][rf].emplace_back(el, rf, test);
        }
      }
      
      // sort endpoints
      std::sort(_endpoints[el][rf].begin(), _endpoints[el][rf].end());

      // update the worst negative slack (wns)
      if(!_endpoints[el][rf].empty()) {
        _wns[el][rf] = _endpoints[el][rf].front().slack();
      }
      else {
        _wns[el][rf] = std::nullopt;
      }

      // update the tns, and fep
      if(!_endpoints[el][rf].empty()) {
        _tns[el][rf] = 0.0f;
        _fep[el][rf] = 0;
        for(const auto& ept : _endpoints[el][rf]) {
          if(auto slack = ept.slack(); slack < 0.0f) {
            _tns[el][rf] = *_tns[el][rf] + slack;
            _fep[el][rf] = *_fep[el][rf] + 1; 
          }
        }
      }
      else {
        _tns[el][rf] = std::nullopt;
        _fep[el][rf] = std::nullopt;
      }
    });
  }

  // run tasks
  _executor.run(_taskflow).wait();
  _taskflow.clear();

  _insert_state(EPTS_UPDATED);
}

// Function: tns
// Update the total negative slack for any transition and timing split. The procedure applies
// the parallel reduction to compute the value.
std::optional<float> Timer::report_tns(std::optional<Split> el, std::optional<Tran> rf) {

  std::scoped_lock lock(_mutex);

  _update_endpoints();

  std::optional<float> v;

  if(!el && !rf) {
    FOR_EACH_EL_RF_IF(s, t, _tns[s][t]) {
      v = !v ? _tns[s][t] : *v + *(_tns[s][t]);
    }
  }
  else if(el && !rf) {
    FOR_EACH_RF_IF(t, _tns[*el][t]) {
      v = !v ? _tns[*el][t] : *v + *(_tns[*el][t]);
    }
  }
  else if(!el && rf) {
    FOR_EACH_EL_IF(s, _tns[s][*rf]) {
      v = !v ? _tns[s][*rf] : *v + *(_tns[s][*rf]);
    }
  }
  else {
    v = _tns[*el][*rf];
  }

  return v;
}

// Function: wns
// Update the total negative slack for any transition and timing split. The procedure apply
// the parallel reduction to compute the value.
std::optional<float> Timer::report_wns(std::optional<Split> el, std::optional<Tran> rf) {

  std::scoped_lock lock(_mutex);

  _update_endpoints();

  std::optional<float> v;
  
  if(!el && !rf) {
    FOR_EACH_EL_RF_IF(s, t, _wns[s][t]) {
      v = !v ? _wns[s][t] : std::min(*v, *(_wns[s][t]));
    }
  }
  else if(el && !rf) {
    FOR_EACH_RF_IF(t, _wns[*el][t]) {
      v = !v ? _wns[*el][t] : std::min(*v, *(_wns[*el][t]));
    }
  }
  else if(!el && rf) {
    FOR_EACH_EL_IF(s, _wns[s][*rf]) {
      v = !v ? _wns[s][*rf] : std::min(*v, *(_wns[s][*rf]));
    }
  }
  else {
    v = _wns[*el][*rf];
  }

  return v;
}

// Function: fep
// Update the failing end points
std::optional<size_t> Timer::report_fep(std::optional<Split> el, std::optional<Tran> rf) {
  
  std::scoped_lock lock(_mutex);

  _update_endpoints();

  std::optional<size_t> v;

  if(!el && !rf) {
    FOR_EACH_EL_RF_IF(s, t, _fep[s][t]) {
      v = !v ? _fep[s][t] : *v + *(_fep[s][t]);
    }
  }
  else if(el && !rf) {
    FOR_EACH_RF_IF(t, _fep[*el][t]) {
      v = !v ? _fep[*el][t] : *v + *(_fep[*el][t]);
    }
  }
  else if(!el && rf) {
    FOR_EACH_EL_IF(s, _fep[s][*rf]) {
      v = !v ? _fep[s][*rf] : *v + *(_fep[s][*rf]);
    }
  }
  else {
    v = _fep[*el][*rf];
  }

  return v;
}

// Function: leakage_power
std::optional<float> Timer::report_leakage_power() {
  std::scoped_lock lock(_mutex);
  _update_power();
  return _leakage_power;
}

// Function: area
// Sum up the area of each gate in the design.
std::optional<float> Timer::report_area() {
  std::scoped_lock lock(_mutex);
  _update_area();
  return _area;
}
    
// Procedure: _enable_full_timing_update
void Timer::_enable_full_timing_update() {
  _insert_state(FULL_TIMING);
}

// Procedure: _insert_full_timing_frontiers
void Timer::_insert_full_timing_frontiers() {

  // insert all zero-fanin pins to the frontier list
  for(auto& kvp : _pins) {
    _insert_frontier(kvp.second);
  }

  // clear the rc-net update flag
  for(auto& kvp : _nets) {
    kvp.second._rc_timing_updated = false;
  }
}

// Procedure: _insert_frontier
void Timer::_insert_frontier(Pin& pin) {
  
  if(pin._frontier_satellite) {
    return;
  }

  pin._frontier_satellite = _frontiers.insert(_frontiers.end(), &pin);
  
  // reset the scc.
  if(pin._scc) {
    _remove_scc(*pin._scc);
  }
}

// Procedure: _remove_frontier
void Timer::_remove_frontier(Pin& pin) {
  if(pin._frontier_satellite) {
    _frontiers.erase(*pin._frontier_satellite);
    pin._frontier_satellite.reset();
  }
}

// Procedure: _clear_frontiers
void Timer::_clear_frontiers() {
  for(auto& ftr : _frontiers) {
    ftr->_frontier_satellite.reset();
  }
  _frontiers.clear();
}

// Procedure: _insert_scc
SCC& Timer::_insert_scc(std::vector<Pin*>& cands) {
  
  // create scc only of size at least two
  auto& scc = _sccs.emplace_front(std::move(cands));
  scc._satellite = _sccs.begin();

  return scc;
}

// Procedure: _remove_scc
void Timer::_remove_scc(SCC& scc) {
  assert(scc._satellite);
  scc._clear();
  _sccs.erase(*scc._satellite); 
}

// Function: report_at   
// Report the arrival time in picoseconds at a given pin name.
std::optional<float> Timer::report_at(const std::string& name, Split m, Tran t) {
  std::scoped_lock lock(_mutex);
  return _report_at(name, m, t);
}

// Function: _report_at
std::optional<float> Timer::_report_at(const std::string& name, Split m, Tran t) {
  _update_timing();
  if(auto itr = _pins.find(name); itr != _pins.end() && itr->second._at[m][t]) {
    return itr->second._at[m][t]->numeric;
  }
  else return std::nullopt;
}

// Function: report_rat
// Report the required arrival time in picoseconds at a given pin name.
std::optional<float> Timer::report_rat(const std::string& name, Split m, Tran t) {
  std::scoped_lock lock(_mutex);
  return _report_rat(name, m, t);
}

// Function: _report_rat
std::optional<float> Timer::_report_rat(const std::string& name, Split m, Tran t) {
  _update_timing();
  if(auto itr = _pins.find(name); itr != _pins.end() && itr->second._at[m][t]) {
    return itr->second._rat[m][t];
  }
  else return std::nullopt;
}

// Function: report_slew
// Report the slew in picoseconds at a given pin name.
std::optional<float> Timer::report_slew(const std::string& name, Split m, Tran t) {
  std::scoped_lock lock(_mutex);
  return _report_slew(name, m, t);
}

// Function: _report_slew
std::optional<float> Timer::_report_slew(const std::string& name, Split m, Tran t) {
  _update_timing();
  if(auto itr = _pins.find(name); itr != _pins.end() && itr->second._slew[m][t]) {
    return itr->second._slew[m][t]->numeric;
  }
  else return std::nullopt;
}

// Function: report_slack
std::optional<float> Timer::report_slack(const std::string& pin, Split m, Tran t) {
  std::scoped_lock lock(_mutex);
  return _report_slack(pin, m, t);
}

// Function: _report_slack
std::optional<float> Timer::_report_slack(const std::string& pin, Split m, Tran t) {
  _update_timing();
  if(auto itr = _pins.find(pin); itr != _pins.end()) {
    return itr->second.slack(m, t);
  }
  else return std::nullopt;
}

// Function: report_load
// Report the load at a given pin name
std::optional<float> Timer::report_load(const std::string& name, Split m, Tran t) {
  std::scoped_lock lock(_mutex);
  return _report_load(name, m, t);
}

// Function: _report_load
std::optional<float> Timer::_report_load(const std::string& name, Split m, Tran t) {
  _update_timing();
  if(auto itr = _nets.find(name); itr != _nets.end()) {
    return itr->second._load(m, t);
  }
  else return std::nullopt;
}

// Function: set_at
Timer& Timer::set_at(std::string name, Split m, Tran t, std::optional<float> v) {

  std::scoped_lock lock(_mutex);

  auto task = _taskflow.emplace([this, name=std::move(name), m, t, v] () {
    if(auto itr = _pis.find(name); itr != _pis.end()) {
      _set_at(itr->second, m, t, v);
    }
    else {
      OT_LOGE("can't set at (PI ", name, " not found)");
    }
  });

  _add_to_lineage(task);

  return *this;
}

// Procedure: _set_at
void Timer::_set_at(PrimaryInput& pi, Split m, Tran t, std::optional<float> v) {
  pi._at[m][t] = v;
  _insert_frontier(pi._pin);
}

// Function: set_rat
Timer& Timer::set_rat(std::string name, Split m, Tran t, std::optional<float> v) {

  std::scoped_lock lock(_mutex);
  
  auto op = _taskflow.emplace([this, name=std::move(name), m, t, v] () {
    if(auto itr = _pos.find(name); itr != _pos.end()) {
      _set_rat(itr->second, m, t, v);
    }
    else {
      OT_LOGE("can't set rat (PO ", name, " not found)");
    }
  });

  _add_to_lineage(op);

  return *this;
}

// Procedure: _set_rat
void Timer::_set_rat(PrimaryOutput& po, Split m, Tran t, std::optional<float> v) {
  po._rat[m][t] = v;
  _insert_frontier(po._pin);
}

// Function: set_slew
Timer& Timer::set_slew(std::string name, Split m, Tran t, std::optional<float> v) {

  std::scoped_lock lock(_mutex);
  
  auto task = _taskflow.emplace([this, name=std::move(name), m, t, v] () {
    if(auto itr = _pis.find(name); itr != _pis.end()) {
      _set_slew(itr->second, m, t, v);
    }
    else {
      OT_LOGE("can't set slew (PI ", name, " not found)");
    }
  });

  _add_to_lineage(task);

  return *this;
}

// Procedure: _set_slew
void Timer::_set_slew(PrimaryInput& pi, Split m, Tran t, std::optional<float> v) {
  pi._slew[m][t] = v;
  _insert_frontier(pi._pin);
}

// Function: set_load
Timer& Timer::set_load(std::string name, Split m, Tran t, std::optional<float> v) {

  std::scoped_lock lock(_mutex);
  
  auto task = _taskflow.emplace([this, name=std::move(name), m, t, v] () {
    if(auto itr = _pos.find(name); itr != _pos.end()) {
      _set_load(itr->second, m, t, v);
    }
    else {
      OT_LOGE("can't set load (PO ", name, " not found)");
    }
  });

  _add_to_lineage(task);

  return *this;
}

// Procedure: _set_load
void Timer::_set_load(PrimaryOutput& po, Split m, Tran t, std::optional<float> v) {

  po._load[m][t] = v ? *v : 0.0f;

  // Update the net load
  if(auto net = po._pin._net) {
    net->_rc_timing_updated = false;
  }
  
  // Enable the timing propagation.
  for(auto arc : po._pin._fanin) {
    _insert_frontier(arc->_from);
  }
  _insert_frontier(po._pin);
}

// ------------------------------------------------------------- RepCut Implementation ----------------------------------------------------

void Timer::_cluster_graph() {

  /*
   * ----------------------------------------------------
   * checker:  print _fprop_cands and _bprop_cands to check if they follow topological order
   * result: they follow topological order 
   */
  // std::cout << "_fprop_cands order: " << "\n";
  // for(auto pin : _fprop_cands) {
  //   std::cout << pin->_name << "\n";
  // }
  // std::cout << "_bprop_cands order: " << "\n";
  // for(auto pin : _bprop_cands) {
  //   std::cout << pin->_name << "\n";
  // }
  /*
   * -----------------------------------------------------    
   */

  /*
   * _fprop_cands and _bprop_cands store the topological order corresponding to _taskflow
   * so we can traverse _bprop_cands and then _fprop_cands to cluster all the pins
   */

  // count the number of _sink_pins to determine how many int64_t we need to represent cone_id
  for(auto pin : _bprop_cands) {
    if(pin->_fanin.size() == 0) {
      _sink_pins.push_back(pin);
    }
  }

  std::vector<Pin*> check_vector;
  for(auto pin : _bprop_cands) {
    if(pin->_btask && pin->_btask.value().num_successors() == 0) {
      check_vector.push_back(pin);
    }
  }

  /*
   * ----------------------------------------------------
   * checker: to see if pin->_fanin == 0 must be sink pins
   * result: for current test cases, the above statement is true
   */
  // int count = 0;
  // for(auto pin : _bprop_cands) {
  //   if(pin->_btask && pin->_btask.value().num_successors() == 0) {
  //     count ++;
  //     std::cout << pin->_name << "\n";
  //   }
  // }
  // if(count != _sink_pins.size()) {
  //   std::cerr << "error: sink pins number is wrong.\n";
  //   std::exit(EXIT_FAILURE);
  // }
  /*
   * -----------------------------------------------------    
   */

  // assign fixed length to cone_id for each pin
  size_t length = _sink_pins.size() / 32 + 1;
  for(auto pin : _frontiers) {
    for(size_t i=0; i<length; i++) {
      pin->_fcone_id.push_back(0);
      pin->_bcone_id.push_back(0);
    } 
  } 

  // assign bcone_id to each sink pins
  int leftshift_count = 0; // counter of how many bit to left shift 
  int next_uint = 0; // counter to indicate which uint to left shift
  for(auto pin : _sink_pins) {
    if(leftshift_count == 32) { // this means the current uint has been filled up
      next_uint ++; // switch to next unit
      pin->_bcone_id[next_uint] = 1; // initialize next unit as 1
      leftshift_count = 1; // reset leftshift_count as 1
    }
    else {
      pin->_bcone_id[next_uint] = 1 << leftshift_count;
      leftshift_count ++;  
    }
  }

  /*
   * ----------------------------------------------------
   * checker: to see if bcone_id is initialized correctly  
   * result: it works for unit32_t, but not uint64_t, uint64_t will still overflow after 1 << 32, quiz?? 
   */
  // for(auto pin : _sink_pins) {
  //   std::cerr << pin->_name << ": ";
  //   for(int i=0; i<pin->_bcone_id.size(); i++) {
  //     _bin_uint32(pin->_bcone_id[i]);
  //     std::cerr << " ";
  //   }
  //   std::cerr << "\n";
  // }
  /*
   * -----------------------------------------------------
   */

  /*
   * what is 
   * if(arc->_has_state(Arc::LOOP_BREAKER)) {
   *  continue;
   * }
   * if(auto& from = arc->_from; from._has_state(Pin::FPROP_CAND))
   * doing? 
   * why wb_dma will fail only sometimes????
   */


  // reversely traverse _bprop_cands to assign _bcone_id for each pin
  for(auto it = _bprop_cands.rbegin(); it != _bprop_cands.rend(); ++it) {
    // assign the dependent pins with bcone_id of current pin 
    for(auto arc : (*it)->_fanout) {
      Pin* to = &(arc->_to);
      if(to->_has_state(Pin::BPROP_CAND)) {
        for(size_t i=0; i<to->_bcone_id.size(); i++) {
          to->_bcone_id[i] = to->_bcone_id[i] | (*it)->_bcone_id[i];
        }
      }
    }
  }

  // for _end_pins (the pins seperates the _ftask and _btask), their _bcone_id and _fconeid are the same
  for(auto pin : _fprop_cands) {
    if(pin->_fanout.size() == 0) {
      pin->_fcone_id = pin->_bcone_id;
      // std::cout << "end_pins: " << pin->_name << "\n";
    }
  }

  // reversely traverse _fprop_cands to assign _fcone_id for each pin
  for(auto it = _fprop_cands.rbegin(); it != _fprop_cands.rend(); ++it) {
    // assign the dependent pins with bcone_id of current pin 
    for(auto arc : (*it)->_fanin) {
      Pin* from = &(arc->_from);
      if(from->_has_state(Pin::FPROP_CAND)) {
        for(size_t i=0; i<from->_fcone_id.size(); i++) {
          from->_fcone_id[i] = from->_fcone_id[i] | (*it)->_fcone_id[i];
        }
      }
    }
  }


  /*
   * ----------------------------------------------------
   * checker: to see if cone_id for each pin is assigned correctly  
   * result: it works for unit32_t, but not uint64_t, uint64_t will still overflow after 1 << 32, quiz?? 
   */
  // for(auto pin : _frontiers) {
  //   std::cout << pin->_name << ": ";
  //   std::cout << "_bcone_id: \n";
  //   for(int i=0; i<pin->_bcone_id.size(); i++) {
  //     _bin_uint32(pin->_bcone_id[i]);
  //     std::cout << " ";
  //   }
  //   std::cout << "\n";
  //   std::cout << "_fcone_id: \n";
  //   for(int i=0; i<pin->_fcone_id.size(); i++) {
  //     _bin_uint32(pin->_fcone_id[i]);
  //     std::cout << " ";
  //   }
  //   std::cout << "\n";
  // }
  /*
   * -----------------------------------------------------
   */

  auto start = std::chrono::steady_clock::now();
  _get_pin_clusters();
  auto end = std::chrono::steady_clock::now();
  sorting_runtime += std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();

  /*
   * ----------------------------------------------------
   * checker: check if _pin_clusters is correct  
   * result: result is correct for c17 
   */
  // for(auto v : _pin_clusters) {
  //   std::cout << "cluster: ";
  //   for(auto pin : v) {
  //     std::cout << pin->_name << " ";
  //   }
  //   std::cout << "\n";
  // }
  /*
   * -----------------------------------------------------
   */

}

void Timer::_get_partition() {
  
  std::vector<size_t> pin_clusters_weights;
  std::vector<size_t> node_clusters_indices;
  std::vector<size_t> edge_clusters_indices;
  size_t index = 0; size_t check_bcone = 0; // 2 counters for filling up node_clusters_indices 
                                                         // and edge_clusters_indices; 
  for(const auto& cluster : _pin_clusters) {
    
    /*
     * fill up pin_cluster_weights
     */
    size_t cluster_runtime = 0;
    for (const auto& pin : cluster) {
      if(pin->_ftask) {
        cluster_runtime += pin->_ftask_runtime;
      }
      if(pin->_btask) {
        cluster_runtime += pin->_btask_runtime;
      }
    }
    pin_clusters_weights.push_back(cluster_runtime);
  
    /*
     * fill up node_clusters_indices and edge_clusters_indices
     */
    if(check_bcone <= _num_bcone_id) {
      if(_is_node_cluster(cluster[0], true)) {
        node_clusters_indices.push_back(index);
      }
      else {
        edge_clusters_indices.push_back(index);
      }
    }
    else {
      if(_is_node_cluster(cluster[0], false)) {
        node_clusters_indices.push_back(index);
      }
      else {
        edge_clusters_indices.push_back(index);
      }
    }
    check_bcone ++; 
    index ++;
  }

//  std::cout << "node_clusters_indices: ";
//  for(auto index : node_clusters_indices) {
//    std::cerr << index << " ";
//  }
//  std::cerr << "\n";
//  std::cerr << "bcone_id of founded_node_clusters: \n";
//  for(auto index : node_clusters_indices) {
//    std::cerr << _pin_clusters[index][0]->_name << ": ";
//    for(auto bits : _pin_clusters[index][0]->_bcone_id) {
//      _bin_uint32(bits);
//      std::cerr << " ";
//    }
//    std::cerr << "\n";
//  }
//  std::cerr << "\n";
  
  // normalize it for mt-kahypar
  size_t max = *(std::max_element(pin_clusters_weights.begin(), pin_clusters_weights.end()));
  size_t min = *(std::min_element(pin_clusters_weights.begin(), pin_clusters_weights.end()));
  
  for(auto& weight : pin_clusters_weights) {
    if(max == min) {
      weight = 1;
    }
    else {
      weight = (weight - min)/(max - min)*2 + 1;
    }
  }

  /*
   * ----------------------------------------------------
   * checker: check if weights of clusters are initialized correctly(no < 0)  
   * result: for des_perf(or most testbench circuit I believe), the weights are 
   * always 1000, and lots of 1, cuz there will always be one super big clusters and lots of small
   * clusters comparing to it. I am not sure if it is a good idea to normalize in this way???
   * we can also do something else like just dividied the weight by 1000 and add 1, in this case 
   * we can see a result of 1350, 1, ... 1, 44, in des_perf case,  at least something different
   */
  // std::cout << "pin_clusters_weights: ";
  // for(auto weight : pin_clusters_weights) {
  //   std::cout << weight << " ";
  // }
  // std::cout << "\n";
  /*
   * -----------------------------------------------------
   */


  std::vector<std::vector<Pin*>> node_clusters;  
  std::vector<std::vector<Pin*>> edge_clusters;
  for(const auto& index : node_clusters_indices) {
    node_clusters.push_back(_pin_clusters[index]);
  }
  for(const auto& index : edge_clusters_indices) {
    edge_clusters.push_back(_pin_clusters[index]);
  }


  /*
   * ----------------------------------------------------
   * checker: check if node_clusters and edge_clusters are correct  
   * result: result is correct for c17 
   */
  // std::cout << "_pin_clusters: \n";
  // for(auto v : _pin_clusters) {
  //   for(auto pin : v) {
  //     std::cout << pin->_name << " ";
  //   }
  //   std::cout << "\n";
  // }
  // std::cout << "node_clusters: \n";
  // for(auto v : node_clusters) {
  //   for(auto pin : v) {
  //     std::cout << pin->_name << " ";
  //   }
  //   std::cout << "\n";
  // }
  // std::cout << "edge_clusters: \n";
  // for(auto v : edge_clusters) {
  //   for(auto pin : v) {
  //     std::cout << pin->_name << " ";
  //   }
  //   std::cout << "\n";
  // }
  /*
   * -----------------------------------------------------
   */

  // these 2 are not accurate, there will be missing nodes and edges which will be added below
  const mt_kahypar_hypernode_id_t num_nodes = node_clusters_indices.size();
  const mt_kahypar_hyperedge_id_t num_hyperedges = edge_clusters_indices.size();

  /* ----------------------------------------------------
   * get hyperedges and hyperedge_indices 
   * ----------------------------------------------------
   */
  /*
   * store a std::vector<Pin*> map representing the mapping between a pin id to a hypergraph node id(0 -> num_nodes-1),
   * this pin belongs to a node cluster
   */
  std::vector<Pin*> map_cone_to_node;
  for(const auto& cluster : node_clusters) {
    map_cone_to_node.push_back(cluster[0]);
  }

  /* 
   * traverse edge_clusters, check if a pin of an edge cluster has the same cone id as the pin in map_cone_to_node
   * 1. to decide cone_id is forward or backward, use _num_bcone_id and edge_clusters_indices to judge
   *    if edge_clusters_indices[i] <= _num_bcone_id, use pin->_bcone_id, else pin->_fcone_id
   * 2. a cone_id of a pin in edge clusters will actually include more than one cone id of a pin in node clusters by definition
   *    so we use a bit series with only one bit = 1 to scan this edge cone_id to get node cone_id 
   */
  std::vector<size_t> eind;
  std::vector<size_t> eptr(num_hyperedges+1, 0);
  for(size_t index=0; index<edge_clusters.size(); index++) {
    
    // get edge cone_id
    auto pin = edge_clusters[index][0];
    std::vector<uint32_t> cone_id;
    if(edge_clusters_indices[index] <= _num_bcone_id) {
      cone_id = pin->_bcone_id;
    }
    else {
      cone_id = pin->_fcone_id;
    }

    // scan edge cone_id to get node cone_id, this nested for-loop is so redundent???????
    std::vector<uint32_t> scan(cone_id.size());
    for(size_t next_uint=0; next_uint<cone_id.size(); next_uint++) {  
      for(size_t leftshift_count=0; leftshift_count<32; leftshift_count++) {
        scan[next_uint] = 1 << leftshift_count;
        uint32_t result = scan[next_uint] & cone_id[next_uint];
        if(result != 0) { // means a node cone id is found
          // then go to map_cone_to_node to find which pin->_bcone_id[next_uint] == result, once found, add its location
          // in map_cone_to_node to hyperedges, only check _bcone_id cuz cone_id for node clusters are all _bcone_id
          for(size_t location =0; location<map_cone_to_node.size(); location++) {
            if(map_cone_to_node[location]->_bcone_id[next_uint] == result) {
              eind.push_back(location);
              eptr[index+1]++;
            }
          }
        }
      }  
    }
  }

  /*
   * there may be a case where there is a isolated small graph with only one sink pin in the original _taskflow
   * in this case, this small graph is a node cluster, it will appear in the _pin_clusters but there will be no
   * pins in edge_clusters whose cone id includes this node cluster
   * we need to check if there is(yes sir) such a case after we construct eptr and eind normally
   */
  // traverse edge_clusters
  std::vector<uint32_t> bit_checker; // checker to see if there is such a node cluster mentioned above
  for(const auto& num : edge_clusters[0][0]->_bcone_id) {
    bit_checker.push_back(num);
  }
  size_t num_uint32 = bit_checker.size();
  for(const auto& cluster : edge_clusters) {
    for(size_t i=0; i<num_uint32; i++) {
      bit_checker[i] = bit_checker[i] | cluster[0]->_bcone_id[i];
      bit_checker[i] = bit_checker[i] | cluster[0]->_fcone_id[i];
    }
  }

  // from LSB to _sink_pins.size(), check if each bit is 1, if not, record its location
  std::vector<uint32_t> scan_result;
  std::vector<uint32_t> scan_bit(bit_checker.size());
  size_t scan_uint = 0; // start scanning from 0 to bit_checker.size()-1 uint32
  for(size_t ls=0; ls<_sink_pins.size(); ls++) { // use ls(left shift) scanner
    scan_bit[scan_uint] = 1 << (ls % 32);
    assert(scan_uint <= bit_checker.size());
    if((bit_checker[scan_uint] & scan_bit[scan_uint]) != scan_bit[scan_uint]) { // find a standalone node cluster
      scan_result.push_back(ls); // store the location of "0" in bit_checker (within _sink_pins.size())
    }
    if((ls % 32 == 0)&&(ls != 0)) { // once finish scanning an uint32_t, go to next one
      scan_uint++;
    }
  }

  // find the corresponding node cluster using scan result
  std::vector<uint32_t> standalone_node_clusters; // store the indices of standalone node clusters in node_clusters/map_cone_to_node
  for(const auto& num : scan_result) {
    std::vector<uint32_t> target(bit_checker.size(), 0);
    target[num / 32] = 1 << (num % 32); // this is the cone id of the node cluster we want to find
    for(size_t i=0; i<node_clusters.size(); i++) {
      if(node_clusters[i][0]->_bcone_id == target) { // we only need to check _bcone_id for node_clusters
        standalone_node_clusters.push_back(i);
      }
    }
  }

  std::unique_ptr<mt_kahypar_hyperedge_id_t[]> hyperedges = std::make_unique<mt_kahypar_hyperedge_id_t[]>(eind.size() + standalone_node_clusters.size());  
  for(size_t i=0; i<eind.size(); i++) {
    hyperedges[i] = eind[i];
  }
  // add the standalone node cluster as one hyperedge
  for(size_t i=0; i<standalone_node_clusters.size(); i++) {
    hyperedges[eind.size()+i] = standalone_node_clusters[i]; 
  }

  // same concept as eptr in hmetis
  std::unique_ptr<size_t[]> hyperedge_indices = std::make_unique<size_t[]>(edge_clusters_indices.size() + 1 + standalone_node_clusters.size());
  hyperedge_indices[0] = 0;
  for(size_t i=1; i<eptr.size(); i++) {
    eptr[i] += eptr[i-1];
    hyperedge_indices[i] = eptr[i];
  }
  // add the standalone node cluster as one hyperedge
  if(standalone_node_clusters.size() != 0) {
    for(size_t i=1; i<=standalone_node_clusters.size(); i++) {
      hyperedge_indices[eptr.size()-1+i] = hyperedge_indices[eptr.size()-1+i-1] + 1; 
    }
  }

  assert(eptr.size() == edge_clusters_indices.size() + 1);

  /* ----------------------------------------------------
   * get node_weights 
   * ----------------------------------------------------
   */
  std::unique_ptr<mt_kahypar_hypernode_weight_t[]> node_weights =
    std::make_unique<mt_kahypar_hypernode_weight_t[]>(num_nodes);

  for(size_t index=0; index<node_clusters_indices.size(); index++) {
    node_weights[index] = pin_clusters_weights[node_clusters_indices[index]]; 
  }
  
  

  /* ----------------------------------------------------
   * get hyperedge_weights 
   * ----------------------------------------------------
   */
  std::unique_ptr<mt_kahypar_hyperedge_weight_t[]> hyperedge_weights =
    std::make_unique<mt_kahypar_hyperedge_weight_t[]>(edge_clusters_indices.size() + standalone_node_clusters.size());

  for(size_t index=0; index<edge_clusters_indices.size(); index++) {
    hyperedge_weights[index] = pin_clusters_weights[edge_clusters_indices[index]];   
  }
  // add the standalone node cluster as one hyperedge
  for(size_t i=0; i<standalone_node_clusters.size(); i++) {
    hyperedge_weights[edge_clusters_indices.size()+i] = pin_clusters_weights[node_clusters_indices[standalone_node_clusters[i]]]; 
  }
  
  /*
   * ----------------------------------------------------
   * checker: check if partition paramters are correct  
   */
  //std::cerr << "edge_clusters:\n";
  //int count = 0;
  //for(const auto& cluster : edge_clusters) {
  //  std::cerr << "cluster: ";
  //  if(edge_clusters_indices[count] <= _num_bcone_id) {
  //    for(auto num : cluster[0]->_bcone_id) {
  //      _bin_uint32(num);
  //      std::cerr << " ";
  //    }
  //  }
  //  else {
  //    for(auto num : cluster[0]->_fcone_id) {
  //      _bin_uint32(num);
  //      std::cerr << " ";
  //    }
  //  }
  //  std::cerr << "\n";
  //}
  //std::cerr << "id mapping:\n";
  //for(size_t i=0; i<map_cone_to_node.size(); i++) {
  //  std::cerr << map_cone_to_node[i]->_name << ": " << i << ": ";
  //  for(auto num : map_cone_to_node[i]->_bcone_id) {
  //    _bin_uint32(num); 
  //    std::cerr << " ";
  //  }
  //  std::cerr << "\n";
  //}
  //std::cerr << "hyperedge_indices: ";
  //for(size_t i=0; i<edge_clusters_indices.size() + 1 + standalone_node_clusters.size(); i++) {
  //  std::cerr << hyperedge_indices[i] << " ";
  //} 
  //std::cerr << "\n";
  //std::cerr << "hyperedges:\n";
  //for(size_t i=0; i<eind.size() + standalone_node_clusters.size(); i++) {
  //  std::cerr << hyperedges[i] << " ";
  //} 
  //std::cerr << "\n";
  //std::cerr << "node_weights: \n"; 
  //for(size_t i=0; i<num_nodes; i++) {
  //  std::cerr << node_weights[i] << " ";
  //}
  //std::cerr << "\n";
  //std::cerr << "hyperedges_weights: \n"; 
  //for(size_t i=0; i<edge_clusters_indices.size() + standalone_node_clusters.size(); i++) {
  //  std::cerr << hyperedge_weights[i] << " ";
  //}
  //std::cerr << "\n";
  /*
   * ----------------------------------------------------
   */
 

  /* ----------------------------------------------------
   * do partition 
   * ----------------------------------------------------
   */

  if(_num_partition > _sink_pins.size()) {
    _num_partition = _sink_pins.size();
  }

  // Construct hypergraph for DEFAULT preset
  mt_kahypar_hypergraph_t hypergraph =
    mt_kahypar_create_hypergraph(DEFAULT, num_nodes, num_hyperedges,
      hyperedge_indices.get(), hyperedges.get(),
      hyperedge_weights.get(), node_weights.get());

  // Setup partitioning context
  mt_kahypar_context_t* context = mt_kahypar_context_new();
  mt_kahypar_load_preset(context, DEFAULT /* corresponds to MT-KaHyPar-D */);
  // In the following, we partition a hypergraph into two blocks
  // with an allowed imbalance of 3% and optimize the connective metric (KM1)
  mt_kahypar_set_partitioning_parameters(context,
    _num_partition /* number of blocks */, _im /* imbalance parameter */,
    KM1 /* objective function */, 42 /* seed */);
  // Enable logging
  mt_kahypar_set_context_parameter(context, VERBOSE, "1");

  // Partition Hypergraph
  mt_kahypar_partitioned_hypergraph_t partitioned_hg =
    mt_kahypar_partition(hypergraph, context);

  // Extract Partition
  std::unique_ptr<mt_kahypar_partition_id_t[]> partition =
    std::make_unique<mt_kahypar_partition_id_t[]>(mt_kahypar_num_hypernodes(hypergraph));
  mt_kahypar_get_partition(partitioned_hg, partition.get());
//  std::cout << "partition:          " << std::endl;
  for(size_t i=0; i<mt_kahypar_num_hypernodes(hypergraph); i++) {
//    std::cout << partition[i] << " ";
    _partition.push_back(partition[i]);
  }
//  std::cout << "\n";

  mt_kahypar_free_context(context);
  mt_kahypar_free_hypergraph(hypergraph);
  mt_kahypar_free_partitioned_hypergraph(partitioned_hg);
  /* ----------------------------------------------------
   * append partition results to partition_id of each pin 
   * ----------------------------------------------------
   */
//  for(size_t i=0; i<_sink_pins.size(); i++) {
//    _partition.push_back(0);
//  }

  // assign partition_id for pins in node cluster by map_cone_to_node 
  for(size_t i=0; i<map_cone_to_node.size(); i++) {
    int partition_id = _partition[i]; // here partition_id is left_shift_count
    map_cone_to_node[i]->_bpartition_id = 1 << partition_id;
  }

  // finish assigning partition_id for pins in node_clusters
  for(const auto& cluster : node_clusters) {
    if(cluster.size() > 1) {
      for(size_t i=1; i<cluster.size(); i++) {
        // notice that here pins in _fprop_cands will also be assigned with _bpartition_id
        cluster[i]->_bpartition_id = cluster[i-1]->_bpartition_id;
      }
    }
  }

  // reversely traverse _bprop_cands to assign _bpartition_id for each pin
  for(auto it = _bprop_cands.rbegin(); it != _bprop_cands.rend(); ++it) {
    // assign the dependent pins with bpartition_id of current pin
    for(auto arc : (*it)->_fanout) {
      Pin* to = &(arc->_to);
      if(to->_has_state(Pin::BPROP_CAND)) {
        to->_bpartition_id = to->_bpartition_id | (*it)->_bpartition_id;
      }
    }
  }
  
  // for _end_pins (the pins seperates the _ftask and _btask), their _bpartition_id and _fpartitionid are the same
  for(auto pin : _fprop_cands) {
    if(pin->_fanout.size() == 0) {
      pin->_fpartition_id = pin->_bpartition_id;
    }
  }

  // reversely traverse _fprop_cands to assign _fpartition_id for each pin
  for(auto it = _fprop_cands.rbegin(); it != _fprop_cands.rend(); ++it) {
    // assign the dependent pins with bpartition_id of current pin
    for(auto arc : (*it)->_fanin) {
      Pin* from = &(arc->_from);
      if(from->_has_state(Pin::FPROP_CAND)) {
      from->_fpartition_id = from->_fpartition_id | (*it)->_fpartition_id;
      }
    }
  }

  /*
   * ----------------------------------------------------
   * checker: check if partition_id is assigned correctly  
   * result: result is correct for c17 
   */
  // for(auto pin : _frontiers) {
  //   std::cout << pin->_name << ": ";
  //   std::cout << "_bpartition_id: ";
  //   _bin_uint32(pin->_bpartition_id);
  //   std::cout << "\n";
  //   std::cout << "_fpartition_id: ";
  //   _bin_uint32(pin->_fpartition_id);
  //   std::cout << "\n";
  // }  
  /*
   * ----------------------------------------------------
   */


}

void Timer::_get_topo_order() {

  /*
   * traverse _bprop_cands and _fprop_cands to scan the partition_id for each pin
   * we need to scan _partition.size() times, start traversing from fprop cuz we need top-down order 
   * _bprop_cands and _fprop_cands are already topologically sorted from top to bottom in taskflow
   * once found a pin belongs to a partition, store it into a std::vector<std::vector<Pin*>> _topo_partitioned_pins
   */
  for(int i=0; i<_num_partition; i++) {
    std::vector<Pin*> one_partition_pins;
    uint32_t scan = 1 << i;
    for(auto pin : _fprop_cands) {
      if((pin->_fpartition_id & scan) == scan) {
        one_partition_pins.push_back(pin);
      }
    }
    _ftopo_partitioned_pins.push_back(one_partition_pins); 
  }
  
  for(int i=0; i<_num_partition; i++) {
    std::vector<Pin*> one_partition_pins;
    uint32_t scan = 1 << i;
    for(auto pin : _bprop_cands) {
      if((pin->_bpartition_id & scan) == scan) {
        one_partition_pins.push_back(pin);
      }
    }
    _btopo_partitioned_pins.push_back(one_partition_pins); 
  }
  /*
   * ----------------------------------------------------
   * checker: check if _topo_partitioned_pins is correct  
   * result: result is correct for c17 
   */
  // std::cout << "_ftopo_partitioned_pins: \n"; 
  // for(const auto& partition : _ftopo_partitioned_pins) {
  //   std::cout << "partition: \n";
  //   for(auto pin : partition) {
  //     std::cout << pin->_name << "\n";
  //   }
  // }
  // std::cout << "_btopo_partitioned_pins: \n"; 
  // for(const auto& partition : _btopo_partitioned_pins) {
  //   std::cout << "partition: \n";
  //   for(auto pin : partition) {
  //     std::cout << pin->_name << "\n";
  //   }
  // }
  /*
   * ----------------------------------------------------
   */

  // assign isRep boolean value for each pin in _frontiers
  for(auto pin : _frontiers) {
    if(_popcount(pin->_bpartition_id) > 1) {
      pin->_isbRep = true;
    }
    if(_popcount(pin->_fpartition_id) > 1) {
      pin->_isfRep = true;
    }
  }
  
}

void Timer::_build_partitioned_taskflow() {
  std::vector<tf::Task> ftask(_ftopo_partitioned_pins.size());
  std::vector<tf::Task> btask(_btopo_partitioned_pins.size());
  int index = 0;
  for(auto& pin_vec : _ftopo_partitioned_pins) {
    ftask[index] = _taskflow.emplace([this, &pin_vec] () {
      for(auto pin : pin_vec) {
        // do ftask
        if(pin->_isfRep) {
          int cur_task_status = 0;
          if(pin->_ftask_status.compare_exchange_strong(cur_task_status, 1)) {
            _fprop_rc_timing(*pin); // 5 typical tasks for a p to update timing in forward propagation // most time consuming if p has net
            _fprop_slew(*pin);
            _fprop_delay(*pin);
            _fprop_at(*pin);
            _fprop_test(*pin);
            pin->_ftask_status.store(2, std::memory_order_release);  
          }
          else { while(pin->_ftask_status.load(std::memory_order_acquire) != 2) {} }
        }
        else {
          _fprop_rc_timing(*pin); // 5 typical tasks for a p to update timing in forward propagation // most time consuming if p has net
          _fprop_slew(*pin);
          _fprop_delay(*pin);
          _fprop_at(*pin);
          _fprop_test(*pin); 
        }
      }
    });
    index++;
  }

  index = 0;
  for(auto& pin_vec : _btopo_partitioned_pins) {
    btask[index] = _taskflow.emplace([this, &pin_vec] () {
      for(auto pin : pin_vec) {
        // do btask
        if(pin->_isbRep) {
          int cur_task_status = 0;
          if(pin->_btask_status.compare_exchange_strong(cur_task_status, 1)) {
            _bprop_rat(*pin); // 1 typical task for a p to update timing in backward progragation
            pin->_btask_status.store(2, std::memory_order_release);
          }
          else { while(pin->_btask_status.load(std::memory_order_acquire) != 2) {} }
        }
        else {
          _bprop_rat(*pin); // 1 typical task for a p to update timing in backward progragation
        }
      }
    });
    index++;
  }

  for(size_t i=0; i<_ftopo_partitioned_pins.size(); i++) {
    ftask[i].precede(btask[i]);
  }
}

void Timer::_execute_task_manually() {
//  for(auto& pin_vec : _topo_partitioned_pins) {
//    size_t ftask_count = 0;
//    for(auto pin : pin_vec) {
//      if(ftask_count < _fprop_cands.size()) {
//        std::cerr << "executing ftask pin: " << pin->_name << "\n"; 
//        _fprop_rc_timing(*pin); // 5 typical tasks for a p to update timing in forward propagation // most time consuming if p has net
//        _fprop_slew(*pin);
//        _fprop_delay(*pin);
//        _fprop_at(*pin);
//        _fprop_test(*pin);
//        ftask_count++;
//      }
//      else {
//        std::cerr << "executing btask pin: " << pin->_name << "\n"; 
//        _bprop_rat(*pin); // 1 typical task for a p to update timing in backward progragation
//      }
//    }
//  } 
//
//  // checker: check if topological order found is correct
//  // set only 1 partition and 1 thread
//  std::vector<Pin*> check_vector_topo;
//  for(auto pin : _fprop_cands) {
//    check_vector_topo.push_back(pin);
//    std::cerr << "checking ftask pin: " << pin->_name << "\n"; 
//  }
//  for(auto pin : _bprop_cands) {
//    check_vector_topo.push_back(pin);
//    std::cerr << "checking btask pin: " << pin->_name << "\n"; 
//  }
//
//  for(size_t i=0; i<check_vector_topo.size(); i++) {
//    assertm(check_vector_topo[i] == _topo_partitioned_pins[0][i], "comparing the address of pin");
//  }

  /*
  std::cout << "_frontiers.size: " << _frontiers.size() << "\n";
  for(auto pin : _fprop_cands) {
    std::cerr << "executing ftask pin: " << pin->_name << "\n"; 
    _fprop_rc_timing(*pin); // 5 typical tasks for a p to update timing in forward propagation // most time consuming if p has net
    _fprop_slew(*pin);
    _fprop_delay(*pin);
    _fprop_at(*pin);
    _fprop_test(*pin);
  }
  for(auto pin : _bprop_cands) {
    std::cerr << "executing btask pin: " << pin->_name << "\n"; 
    _bprop_rat(*pin); // 1 typical task for a p to update timing in backward progragation
  }
  */
}

std::string Timer::_uint32_to_string(const std::vector<uint32_t>& vec) {
  std::string result = "_";
  for(const auto& num : vec) {
    result += std::to_string(num) + "_";
  }
  return result;
}

void Timer::_get_pin_clusters() {

  auto start = std::chrono::steady_clock::now();
  // make a copy of prop_cands 
  std::vector<Pin*> fprop_cands_copy;
  std::vector<Pin*> bprop_cands_copy;
  for(auto pin : _fprop_cands) {
    fprop_cands_copy.push_back(pin);
  }
  for(auto pin : _bprop_cands) {
    bprop_cands_copy.push_back(pin);
  }
  auto end = std::chrono::steady_clock::now();
  copy_runtime += std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
//
//  start = std::chrono::steady_clock::now();
//  // sort bprop_cands_copy
//  _quicksort(bprop_cands_copy, 0, bprop_cands_copy.size()-1, true);
//  // sort fprop_cands_copy
//  _quicksort(fprop_cands_copy, 0, fprop_cands_copy.size()-1, false);
//  end = std::chrono::steady_clock::now();
//  quick_sort_runtime += std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
//
  /*
  std::vector<Pin*> fprop_cands_copy;
  std::vector<Pin*> bprop_cands_copy;
  std::sort(bprop_cands_copy.begin(), bprop_cands_copy.end(), [this] (Pin* a, Pin* b) {
    return _comp_uint32_vector(a->_bcone_id, b->_bcone_id); 
  });
  std::sort(fprop_cands_copy.begin(), fprop_cands_copy.end(), [this] (Pin* a, Pin* b) {
    return _comp_uint32_vector(a->_fcone_id, b->_fcone_id); 
  });
  */
//
//  // traverse bprop_cands_copy to get _pin_clusters
//  _pin_clusters.push_back({bprop_cands_copy[0]});
//  int index_pin_cluster = 0;
//  for(size_t i=1; i<bprop_cands_copy.size(); i++) {
//    if(bprop_cands_copy[i]->_bcone_id == bprop_cands_copy[i-1]->_bcone_id) {
//      _pin_clusters[index_pin_cluster].push_back(bprop_cands_copy[i]);
//    }
//    else{
//      _pin_clusters.push_back({bprop_cands_copy[i]});
//      index_pin_cluster++;
//    }
//  }
//  _num_bcone_id = _pin_clusters.size() - 1; // record _num_bcone_id for later filling up node_clusters and edge_clusters
//                                            // _fcone_id will overlapped with _bcone_id, but it does not matter(-1 cuz index starts from 0)
//
//  std::vector<std::vector<uint32_t>> visited_cone;
//  for(const auto& cluster : _pin_clusters) {
//    visited_cone.push_back(cluster[0]->_bcone_id);
//  }
 
  // traverse bprop_cands to fill up _pin_clusters
  std::vector<std::vector<uint32_t>> visited_cone;
  for(size_t i=0; i<bprop_cands_copy.size(); i++) {
    auto it = std::find(visited_cone.begin(), visited_cone.end(), bprop_cands_copy[i]->_bcone_id);
    if(it == visited_cone.end()) { 
      _pin_clusters.push_back({bprop_cands_copy[i]});
      visited_cone.push_back(bprop_cands_copy[i]->_bcone_id); 
    }
    else {
      _pin_clusters[std::distance(visited_cone.begin(), it)].push_back(bprop_cands_copy[i]);
    }
  }
  _num_bcone_id = _pin_clusters.size() - 1; // record _num_bcone_id for later filling up node_clusters and edge_clusters
                                            // _fcone_id will overlapped with _bcone_id, but it does not matter(-1 cuz index starts from 0)

  // traverse fprop_cands_copy to complete _pin_clusters
  for(size_t i=0; i<fprop_cands_copy.size(); i++) {
    auto it = std::find(visited_cone.begin(), visited_cone.end(), fprop_cands_copy[i]->_fcone_id);
    if(it == visited_cone.end()) { 
      _pin_clusters.push_back({fprop_cands_copy[i]});
      visited_cone.push_back(fprop_cands_copy[i]->_fcone_id); 
    }
    else {
      _pin_clusters[std::distance(visited_cone.begin(), it)].push_back(fprop_cands_copy[i]);
    }
  }

  /*
   * ----------------------------------------------------
   * checker: check if _pin_clusters sorted is correct  
   */
  // std::cerr << "check _pin_clusters: \n";
  // for(auto cluster : _pin_clusters) {
  //   std::cerr << cluster[0]->_name << ": ";
  //   std::string s = _uint32_to_string(cluster[0]->_bcone_id);
  //   std::cerr << s << "\n";
  // }

  // std::cerr << "check visited_cone: \n";
  // for(auto cone_id : visited_cone) {
  //   std::string s = _uint32_to_string(cone_id);
  //   std::cerr << s << "\n";
  // }

  /*
   * ----------------------------------------------------
   */
}

void Timer::_quicksort(std::vector<Pin*>& pin_cands, int low, int high, bool isbprop) {

  if(low < high) {
    int pi = _partition_qs(pin_cands, low, high, isbprop);
    _quicksort(pin_cands, low, pi-1, isbprop);
    _quicksort(pin_cands, pi+1, high, isbprop);
  }

}

int Timer::_partition_qs(std::vector<Pin*>& pin_cands, int low, int high, bool isbprop) {

  // Choosing the pivot
  Pin* pivot = pin_cands[high];

  // Index of smaller element and indicates
  // the right position of pivot found so far
  int i = (low - 1);

  for (int j = low; j <= high - 1; j++) {
    // If current element is smaller than the pivot
    if(isbprop) {
      if (_comp_uint32_vector(pin_cands[j]->_bcone_id, pivot->_bcone_id)) {
        // Increment index of smaller element
        i++;
        std::swap(pin_cands[i], pin_cands[j]);
      }
    }
    else {
      if (_comp_uint32_vector(pin_cands[j]->_fcone_id, pivot->_fcone_id)) {
        // Increment index of smaller element
        i++;
        std::swap(pin_cands[i], pin_cands[j]);
      }
    }
  }
  std::swap(pin_cands[i + 1], pin_cands[high]);
  return (i + 1);
};

bool Timer::_comp_uint32_vector(const std::vector<uint32_t>& a, const std::vector<uint32_t>& b) {

  int length = a.size();
  assert(a.size() != 0);
  int count = length - 1;
  while(count >= 0) {
    if(a[count] < b[count]) {
      return true;
    }
    count --;
  }
  return false;
}

void Timer::_bin_uint32(uint32_t n) {
  /*
  uint32_t i;
  std::cerr << "0";
  for (i = 1 << 30; i > 0; i = i / 2)
  {
    if((n & i) != 0)
    {
      std::cerr << "1";
    }
    else
    {
      std::cerr << "0";
    }
  }
  */

  std::bitset<32> b(n);
  std::cerr << b;

}

bool Timer::_is_node_cluster(const Pin* p, bool isbcone) {
  
  // traverse the cone_id (std::vector<uint32_t>)
  uint32_t result = 0;
  std::vector<uint32_t> cone_id;
  if(isbcone) {
    cone_id = p->_bcone_id;
  }
  else {
    cone_id = p->_fcone_id;
  }
  for(auto bits : cone_id) {
    result += _popcount(bits);
  } 
  if(result == 1) { // only return true when there is only one uint32_t is the power of 2
                   // i.e, there is only one bit = 1 in the bit series of cone_id for this pin
    return true;
  }
  else {
    return false;
  }
}

void Timer::_reset_repcut() {
  
  _sink_pins.clear();
  
  for(auto pin : _frontiers) {
    pin->_fcone_id.clear();
    pin->_bcone_id.clear();
    pin->_fpartition_id = 0;
    pin->_bpartition_id = 0;
    pin->_isfRep = false;
    pin->_isbRep = false;
    pin->_ftask_status = 0;
    pin->_btask_status = 0;
  }

  _pin_clusters.clear();

  _partition.clear();

  _ftopo_partitioned_pins.clear();
  _btopo_partitioned_pins.clear();

  _num_ftask_partitions.clear();

  _num_bcone_id = 0;
}

uint32_t Timer::_popcount(uint32_t number) {
  uint32_t count = 0;

 //calculate the total set bits in a number
 while (number){
    count += number & 1;
    number >>= 1;
 }
 return count;
}

void Timer::_check_mtkahypar() {

  // Initialize thread pool
  mt_kahypar_initialize_thread_pool(
    std::thread::hardware_concurrency() /* use all available cores */,
    true /* activate interleaved NUMA allocation policy */ );

  // In the following, we construct a hypergraph with 7 nodes and 4 hyperedges
  const mt_kahypar_hypernode_id_t num_nodes = 4;
  const mt_kahypar_hyperedge_id_t num_hyperedges = 3;

  // The hyperedge indices points to the hyperedge vector and defines the
  // the ranges containing the pins of each hyperedge
  std::unique_ptr<size_t[]> hyperedge_indices = std::make_unique<size_t[]>(4);
  hyperedge_indices[0] = 0; hyperedge_indices[1] = 2; hyperedge_indices[2] = 3;
  hyperedge_indices[3] = 4; 

  std::unique_ptr<mt_kahypar_hyperedge_id_t[]> hyperedges =
    std::make_unique<mt_kahypar_hyperedge_id_t[]>(4);
  hyperedges[0] = 0;  hyperedges[1] = 2;
  hyperedges[2] = 1;  hyperedges[3] = 3; 
  
  // Define node weights
  std::unique_ptr<mt_kahypar_hypernode_weight_t[]> node_weights =
    std::make_unique<mt_kahypar_hypernode_weight_t[]>(4);
  node_weights[0] = 1; node_weights[1] = 1; node_weights[2] = 1; node_weights[3] = 3;

  // Define hyperedge weights
  std::unique_ptr<mt_kahypar_hyperedge_weight_t[]> hyperedge_weights =
    std::make_unique<mt_kahypar_hyperedge_weight_t[]>(4);
  hyperedge_weights[0] = 1; hyperedge_weights[1] = 1;
  hyperedge_weights[2] = 3;

  // Construct hypergraph for DEFAULT preset
  mt_kahypar_hypergraph_t hypergraph =
    mt_kahypar_create_hypergraph(DEFAULT, num_nodes, num_hyperedges,
      hyperedge_indices.get(), hyperedges.get(),
      hyperedge_weights.get(), node_weights.get());

  std::cout << "Number of Nodes            = " << mt_kahypar_num_hypernodes(hypergraph) << std::endl;
  std::cout << "Number of Hyperedges       = " << mt_kahypar_num_hyperedges(hypergraph) << std::endl;
  std::cout << "Number of Pins             = " << mt_kahypar_num_pins(hypergraph) << std::endl;
  std::cout << "Total Weight of Hypergraph = " << mt_kahypar_hypergraph_weight(hypergraph) << std::endl;

  // Setup partitioning context
  mt_kahypar_context_t* context = mt_kahypar_context_new();
  mt_kahypar_load_preset(context, DEFAULT /* corresponds to MT-KaHyPar-D */);
  // In the following, we partition a hypergraph into two blocks
  // with an allowed imbalance of 3% and optimize the connective metric (KM1)
  mt_kahypar_set_partitioning_parameters(context,
    2 /* number of blocks */, 0.03 /* imbalance parameter */,
    KM1 /* objective function */, 42 /* seed */);
  // Enable logging
  mt_kahypar_set_context_parameter(context, VERBOSE, "1");

  // Partition Hypergraph
  mt_kahypar_partitioned_hypergraph_t partitioned_hg =
    mt_kahypar_partition(hypergraph, context);

  // Extract Partition
  std::unique_ptr<mt_kahypar_partition_id_t[]> partition =
    std::make_unique<mt_kahypar_partition_id_t[]>(mt_kahypar_num_hypernodes(hypergraph));
  mt_kahypar_get_partition(partitioned_hg, partition.get());

  // Extract Block Weights
  std::unique_ptr<mt_kahypar_hypernode_weight_t[]> block_weights =
    std::make_unique<mt_kahypar_hypernode_weight_t[]>(2);
  mt_kahypar_get_block_weights(partitioned_hg, block_weights.get());

  // Compute Metrics
  const double imbalance = mt_kahypar_imbalance(partitioned_hg, context);
  const double km1 = mt_kahypar_km1(partitioned_hg);

  // Output Results
  std::cout << "Partitioning Results:" << std::endl;
  std::cout << "Imbalance         = " << imbalance << std::endl;
  std::cout << "Km1               = " << km1 << std::endl;
  std::cout << "Weight of Block 0 = " << block_weights[0] << std::endl;
  std::cout << "Weight of Block 1 = " << block_weights[1] << std::endl;

  mt_kahypar_free_context(context);
  mt_kahypar_free_hypergraph(hypergraph);
  mt_kahypar_free_partitioned_hypergraph(partitioned_hg);
}

};  // end of namespace ot. -----------------------------------------------------------------------




