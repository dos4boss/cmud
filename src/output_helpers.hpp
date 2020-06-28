#pragma once

#include <iostream>

class RAIIStreamReconstructor {
public:
  explicit RAIIStreamReconstructor(std::ostream &_ios)
      : ios(_ios), f(_ios.flags()) {}
  ~RAIIStreamReconstructor() { ios.flags(f); }

  RAIIStreamReconstructor(const RAIIStreamReconstructor &rhs) = delete;
  RAIIStreamReconstructor &operator=(const RAIIStreamReconstructor &rhs) = delete;

private:
  std::ostream &ios;
  std::ios::fmtflags f;
};
