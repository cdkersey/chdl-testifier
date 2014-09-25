#include <fstream>

#include <chdl/chdl.h>
#include <chdl/cassign.h>
#include <chdl/ag.h>
#include <chdl/egress.h>

#include <cmath>

#include "testify-module.h"

TESTIFY_VERSION("2014-09-24");

namespace chdl {
  // Complex arithmetic
  template <typename T> using cpx =
    ag<STP("r"), T,
    ag<STP("i"), T> >;

  template <typename T> cpx<T> LitCpx(double r, double i) {
    cpx<T> rval;
    Lit(_(rval, "r"), r);
    Lit(_(rval, "i"), i);
    
    return rval;
  }

  template <typename T> cpx<T> operator+(const cpx<T> &a, const cpx<T> &b) {
    cpx<T> s;
    _(s, "r") = _(a, "r") + _(b, "r");
    _(s, "i") = _(a, "i") + _(b, "i");
    return s;
  }

  template <typename T> cpx<T> operator*(const cpx<T> &a, const cpx<T> &b) {
    cpx<T> p;
    _(p, "r") = _(a, "r") * _(b, "r") - _(a, "i") * _(b, "i");
    _(p, "i") = _(a, "r") * _(b, "i") + _(a, "i") * _(b, "r");
    return p;
  }

  template <typename T> T Abs2(const cpx<T> &x) {
    return _(x, "r") * _(x, "r") + _(x, "i") * _(x, "i");
  }

  // Fixed point numbers
  template <unsigned W, unsigned F> struct fxp : public bvec<W + F> {
    fxp() {}
    fxp(const bvec<W+F> &v): bvec<W + F>(v) {}
    fxp(const fxp<W, F> &v): bvec<W + F>(v) {}
  };
  
  template <unsigned W, unsigned F> struct sz<fxp<W,F> > {
    const static unsigned value = W+F;
  };

  template<unsigned W, unsigned F> void Lit(fxp<W, F> f, double d) {
    f = fxp<W, F>(Lit<W+F>(d*(1ul<<F) + 0.5));
  }

  template <unsigned W, unsigned F>
    fxp<W, F> operator*(const fxp<W, F> &a, const fxp<W, F> &b)
  {
    bvec<2*(W+F)> ax(Sext<2*(W+F)>(a)), bx(Sext<2*(W+F)>(b));
    return fxp<W, F>((ax * bx)[range<F, 2*F+W-1>()]);
  }

  template <unsigned W, unsigned F>
    fxp<W, F> Reg(const fxp<W, F> &d, double init = 0)
  {
    return Reg(bvec<W+F>(d), long(init*(1ul<<F) + 0.5));
  }
}

using namespace chdl;
using namespace std;

template <unsigned I, typename T> void MandelbrotUnit(
  bvec<CLOG2(I+1)> iters, node strobe, node ld, const cpx<T> &init, double inc
)
{
  cpx<T> next_x0, x0(Reg(next_x0)), next_x, x(Reg(next_x));
  Cassign(next_x).
    IF(strobe, x0).
    ELSE(x*x + x0);
  Cassign(next_x0).
    IF(ld, init).
    IF(strobe, x0 + LitCpx<T>(inc, 0)).
    ELSE(x0);

  TAP(x0); TAP(x);

  T four;
  Lit(four, 4.0);

  node diverge(Abs2(x) > four);
  strobe = diverge || iters == Lit<CLOG2(I+1)>(I);

  bvec<CLOG2(I+1)> next_iters;
  iters = Reg(next_iters);
  Cassign(next_iters).
    IF(strobe || ld, Lit<CLOG2(I+1)>(0)).
    ELSE(iters + Lit<CLOG2(I+1)>(1));
}

void build() {
  typedef fxp<4,8> T;
  cpx<T> init0(LitCpx<T>(-2,-2)), next_init, init(Reg(next_init));
  bvec<7> iters;
  node strobe, ld0, ld;
  MandelbrotUnit<100, T>(iters, strobe, ld || ld0, Mux(ld0,init,init0), 0.04);

  bvec<7> next_col, col(Reg(next_col));
  Cassign(next_col).
    IF(ld || ld0, Lit<7>(0)).
    IF(strobe, col + Lit<7>(1)).
    ELSE(col);

  bvec<7> next_row, row(Reg(next_row));
  Cassign(next_row).
    IF(ld, row + Lit<7>(1)).
    ELSE(row);

  ld0 = Reg(Lit(0), 1);
  ld = strobe && col == Lit<7>(99);

  Cassign(next_init).
    IF(ld0, init0).
    IF(ld, init + LitCpx<T>(0,0.08)).
    ELSE(init);

  TAP(iters);
  TAP(init);
  TAP(strobe);
  TAP(ld);
  TAP(row);
  TAP(col);
}
