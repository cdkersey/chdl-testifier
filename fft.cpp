// Pipelined real-time FFT implementation.
#include <chdl/chdl.h>
#include <chdl/numeric.h>
#include <chdl/lfsr.h>

#include <cstdlib>

#include "testify-module.h"

using namespace chdl;

template <typename T> using cpx = ag<STP("r"), T, ag<STP("i"), T> >;

template <typename T> cpx<T> operator*(const cpx<T> &l, const cpx<T> &r) {
  cpx<T> x;
  _(x, "r") = _(l, "r") * _(r, "r") - _(l, "i") * _(r, "i");
  _(x, "i") = _(l, "r") * _(r, "i") + _(l, "i") * _(r, "r");
  
  return x;
}

template <typename T> cpx<T> operator-(const cpx<T> &l, const cpx<T> &r) {
  cpx<T> x;
  _(x, "r") = _(l, "r") - _(r, "r");
  _(x, "i") = _(l, "i") - _(r, "i");
  return x;
}

template <typename T> cpx<T> operator+(const cpx<T> &l, const cpx<T> &r) {
  cpx<T> x;
  _(x, "r") = _(l, "r") + _(r, "r");
  _(x, "i") = _(l, "i") + _(r, "i");
  return x;
}

template <typename T> vec<1, cpx<T> > fft(const vec<1, cpx<T> > &v)
  { return v; }

template <typename T, unsigned N>
  vec<N, cpx<T> > fft(const vec<N, cpx<T> > &v)
{
  vec<N, cpx<T> > out;
  vec<N/2, cpx<T> > e_in, o_in, e_out, o_out;
  for (unsigned i = 0; i < N; ++i) {
    if (i & 1) o_in[i/2] = v[i];
    else       e_in[i/2] = v[i];
  }

  e_out = fft(e_in);
  o_out = fft(o_in);

  // Perform butterfly
  for (unsigned i = 0; i < N/2; ++i) {
    cpx<T> twiddle, p = twiddle * o_out[i];
    Lit(_(twiddle, "r"), cos(-2*M_PI*i/double(N)));
    Lit(_(twiddle, "i"), sin(-2*M_PI*i/double(N)));
    
    out[i] = e_out[i] + p;
    out[N/2 + i] = e_out[i] - p;
  }

  return out;
}

void build() {
  const unsigned N = 16, W = 6, F = 6;
  
  using namespace std;

  vec<N, cpx<fxp<W, F> > > x, f;

  /* Bad linear congruential generators for each value*/
  for (unsigned i = 0; i < N; ++i) {
    bvec<32> rng;
    rng = Reg(rng * Lit<32>(rand()) + Lit<32>(rand()), rand());
    bvec<W + F> val = Zext<W + F>(rng[range<0, F+1>()]) - Lit<W + F>(1<<F);
    Flatten(_(x[i], "r")) = val;
    _(x[i], "i") = Lit<W + F>(0);
  }

  f = fft(x);

  TAP(x);
  TAP(f);
}

TESTIFY_VERSION("2016-09-26");
