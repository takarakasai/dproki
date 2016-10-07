
#ifndef COMMON_H
#define COMMON_H

#include <string>

class Common {
private:
  static std::string dirpath_;
  static std::string out_path_head_;

public:
  static std::string& DirPath();
  static std::string& OutPathHead();
};

#endif

