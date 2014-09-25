#include <chdl/chdl.h>
#include <chdl/lfsr.h>

#include "testify-module.h"

using namespace chdl;

const unsigned N(1023), T(3);

void build() {
  bvec<8> x(Lfsr<8, N, T, 0x5eed>());

  TAP(x);
}

TESTIFY_VERSION("2014-09-24");
