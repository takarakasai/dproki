
#include "dproki/Common.h"

namespace Dp {

  std::string Common::dirpath_       = "";
  std::string Common::out_path_head_ = "";
  
  std::string& Common::DirPath() {
    return dirpath_;
  }
  
  std::string& Common::OutPathHead() {
    return out_path_head_;
  }

}
