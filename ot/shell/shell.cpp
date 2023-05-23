#include <ot/shell/shell.hpp>

namespace ot {

// Constructor
Shell::Shell(const std::string& w, FILE* is, std::ostream& os, std::ostream& es, size_t num_partitions, size_t num_threads) :
  _os     {os},
  _es     {es},
  _prompt {w, "ot> ", ot::user_home() / ".ot_history", is, os, es} {
 
//  _timer._set_num_partition = num_partitions;
//  _timer._num_threads = num_threads;

  // Add auto-complete
  for(const auto& kvp : _handles) {
    _prompt.autocomplete(kvp.first);
  }

  _prompt.autocomplete("quit");
  _prompt.autocomplete("exit");
}
  
// Operator: ()
void Shell::operator()() {  

  bool quit {false};
  
  while(!quit && _prompt.readline(_line)) {

    // Remove the comment
    if(auto cpos = _line.find('#'); cpos != std::string::npos) {
      _line.erase(cpos);
    }
    
    _op.clear();
    _is.clear();
    _is.str(_line);
    _is >> _op;
    
    // Nothing to do with empty line
    if(_op.empty()) {
    }
    // leave
    else if(_op == "quit" || _op == "exit") {
      quit = true; 
    }
    // built-in command
    else if(auto itr = _handles.find(_op); itr != _handles.end()) {
      (this->*(itr->second))();
    } 
    // undefined command
    else {
      _es << "undefined command: " << std::quoted(_line) << '\n';
    }

    // flush
    _os.flush();
    _es.flush();
  }
}

};  // end of namespace ot. -----------------------------------------------------------------------




