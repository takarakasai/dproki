
#ifndef ERR_CHK_H
#define ERR_CHK_H

#include "dp_type.h"

#define ECALL(function)     \
  do {                      \
    errno_t eno = function; \
    if (eno != 0) {         \
      return eno;           \
    }                       \
  } while(0)

#endif

