#include <chdl/chdl.h>
#include "testify-module.h"

using namespace chdl;

const unsigned N(32);

void build() {
  bvec<N> x, y, z(x * y);

  x = Reg(x + Lit<N>(3), 2);
  y = Reg(y + Lit<N>(5), 1);

  TAP(x); TAP(y); TAP(z);
}

TESTIFY_VERSION("2014-09-23");
