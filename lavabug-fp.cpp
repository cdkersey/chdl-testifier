#include <iostream>
#include <fstream>
#include <cmath>

#include <chdl/chdl.h>
#include <chdl/cassign.h>
#include <chdl/ag.h>
#include <chdl/cassign.h>

#include "testify-module.h"

TESTIFY_VERSION("2014-11-12");

namespace chdl {
  // At some point, we need to introduce signed integer types and maybe even
  // some 1-hot other radix types so we can get beyond our idiotic insistence
  // that everything be either an unsigned integer or hard-to-do. Here's a
  // first attempt, a 2's complement signed integral type. Note that the output
  // types are all large enough to hold any result, so that overflow detection
  // can be done reasonably, but only if required, using the TooBig() function.
  template <unsigned N> struct si : public bvec<N> {
    // Automatic conversion from other bit widths. It's not always easy to
    // predict the bit widths of your result, so we provide this for convenience
    // despite the fact that it can lead to unintentional overflow errors. We
    // hope auto variables and TooBig() will be used when this may be an issue.
    template <unsigned M> si(const si<M> &x): bvec<N>(Zext<N>(x)) {}

    si(const bvec<N> &x): bvec<N>(x) {}

    si(): bvec<N>() {}
  };

  template <unsigned N> si<N + 1> operator+(const si<N> &a, const si<N> &b) {
    return Sext<N+1>(bvec<N>(a)) + Sext<N+1>(bvec<N>(b));
  }

  template <unsigned N> si<N + 1> operator-(const si<N> &x) {
    return Sext<N+1>(~bvec<N>(x)) + Lit<N+1>(1);
  }

  template <unsigned N> si<N + 1> operator-(const si<N> &a, const si<N> &b) {
    return Sext<N+1>(a) + -b;
  }

  template <unsigned N> node operator<(const si<N> &a, const si<N> &b) {
    return bvec<N + 1>(a - b)[N];
  }

  template <unsigned N> node operator>(const si<N> &a, const si<N> &b) {
    return b < a;
  }

  template <unsigned N> node operator<=(const si<N> &a, const si<N> &b) {
    return !(a > b);
  }

  template <unsigned N> node operator>=(const si<N> &a, const si<N> &b) {
    return !(a < b);
  }

  template <unsigned M, unsigned N> node TooBig(si<N> &x) {
    node allOne(!OrN(~x[range<M, N-1>()])), allZero(!OrN(x[range<M, N-1>()]));
    return !allOne && !allZero;
  }
}

namespace chdl {
  // Bias-(2^(E-1)-1) exponent IEEE-like floating point number, including
  // support for Inf/NaN and subnormal numbers.
  template <unsigned E, unsigned M> struct fp :
    public ag<STP("s"), node,
           ag<STP("e"), bvec<E>,
           ag<STP("m"), bvec<M> > > >
  {};

  template <unsigned E, unsigned M> struct sz<fp<E, M> > {
    const static unsigned value = 1 + E + M;
  };

  // For internal representation. Two's complement exponent. Has extra bits to
  // represent larger-than-1 significand. No support for subnormal numbers.
  // NaN/Inf supported.
  template <unsigned E, unsigned N, unsigned M> struct ifp :
    public ag<STP("s"), node,
           ag<STP("e"), bvec<E>,
           ag<STP("n"), bvec<N>,
           ag<STP("m"), bvec<M> > > > >
  {};

  template <unsigned E, unsigned N, unsigned M> struct sz<ifp<E, N, M> > {
    const static unsigned value = 1 + E + N + M;
  };

  // Typedefs for common IEEE floating point types.
  typedef fp< 5, 10> fp16;
  typedef fp< 8, 23> fp32;
  typedef fp<11, 52> fp64;
  typedef fp<15,112> fp128;

  // Literal, value provided by a double. If the CHDL type is higher-precision
  // than a double, the result will be less precise.
  template <unsigned E, unsigned M> fp<E, M> LitF(double d) {
    using namespace std;

    bool s(d < 0);
    if (s) { d = -d; }

    int e;
    unsigned long m;

    if (d == 0) {
      e = 0;
      m = 0;
    } else {
      const unsigned BIAS((1ull<<(E-1))-1);
      const unsigned MAX_E((1ull<<E)-2);

      double e_val(log2(d) + BIAS);
 
      if (e_val < 1) {
        // Subnormal
        e = 0;
        m = d*pow(2,BIAS + M);
      } else if (e_val > MAX_E) {
        // Infinity
        e = MAX_E + 1;
        m = 0;
      } else {
        // Ordinary
        e = e_val;
        m = (d/pow(2,e - BIAS)-1)*pow(2,M);
      }

    }

    fp<E, M> r;
    _(r, "s") = Lit(s);
    _(r, "e") = Lit<E>(e);
    _(r, "m") = Lit<M>(m);

    return r;
  }

  // Literal functions for the common types.
  fp16  LitF16 (double d) { return LitF< 5, 10>(d); }
  fp32  LitF32 (double d) { return LitF< 8, 23>(d); }
  fp64  LitF64 (double d) { return LitF<11, 52>(d); }
  fp128 LitF128(double d) { return LitF<15,112>(d); }

  // Type-inferring overloadedliteral function
  template <unsigned E, unsigned M> void Lit(fp<E, M> f, double d) {
    f = LitF<E, M>(d);
  }

  template <typename T> vec<0, T> Reverse(const vec<0, T> &in) { return in; }
  template <typename T> vec<1, T> Reverse(const vec<1, T> &in) { return in; }
  template <typename T> vec<2, T> Reverse(const vec<2, T> &in) {
    return Cat(in[0], in[1]);
  }

  template <typename T, unsigned N> vec<N, T> Reverse(const vec<N, T> &in) {
    return Cat(Reverse(in[range<0,N-2>()]), vec<1, T>(in[N-1]));
  }

  // Convert an internal representation number to a canonical floating point
  // number. Don't go through pains to make it fit; just unpack it. Normalized
  // numbers all become 1.[...] with the same exponent converted to 2's
  // complement. Subnormal numbers all become 0.[...]e-(BIAS-1).
  template <unsigned E, unsigned N, unsigned M, unsigned EE, unsigned MM>
    ifp<E, N, M> FpToIfp(const fp<EE, MM> &in)
  {
    const unsigned long BIAS((1ull<<(EE-1))-1);

    ifp<E, N, M> r;
    node subnorm(_(in, "e") == Lit<EE>(0)),
         nan(_(in, "e") == ~Lit<EE>(0) && !OrN(_(in, "m"))),
         inf(_(in, "e") == ~Lit<EE>(0) && OrN(_(in, "m")));

    bvec<N + M> nm(Cat(_(r, "n"), _(r, "m")));
    bvec<E> e(_(r, "e"));
    node s(_(r, "s"));

    // Handle subnormal numbers, including +/- 0

    // Add zeros to the end, getting the interesting digits near-ish to the
    // binary point. If we wind up truncating, this also has the benefit of
    // maintaining correctness by removing the LSBs instead of the MSBs.
    bvec<N + M> subnorm_nm =
      Cat(Lit<N>(0), Reverse(Zext<M>(Reverse(_(in, "m")))));
    bvec<E> subnorm_e(Lit<E>(-(BIAS-1)));

    // Handle other values, including NaN and infinity.
    bvec<M> norm_m = Reverse(Zext<M>(Reverse(_(in, "m"))));
    bvec<N + M> norm_nm = Cat(Lit<N>(1), norm_m);
    bvec<E> norm_e(Zext<E>(_(in, "e")) - Lit<E>(BIAS));

    // Assign output values.
    s = _(in, "s"); // Sign is trivial

    Cassign(nm).
      IF(subnorm, subnorm_nm).
      ELSE(norm_nm);

    Cassign(e).
      IF(subnorm, subnorm_e).
      ELSE(norm_e);

    return r;
  }

// This is basically a renormalization stage. We assume that the internal format
// can store all numbers available in the external format.
template <unsigned EE, unsigned MM, unsigned E, unsigned N, unsigned M>
  fp<EE, MM> IfpToFp(const ifp<E, N, M> &in)
{
  const unsigned BIAS((1ull<<(EE-1))-1);

  bvec<N + M> nm(Cat(_(in, "n"), _(in, "m")));

  bvec<CLOG2(N + M)> msb(Log2(nm));

  bvec<E> norm_e(Zext<E>(msb) + _(in, "e") - Lit<E>(M));
  node zero(nm == Lit<N + M>(0)),
       subnorm(zero || si<E>(norm_e) <= si<E>(Lit<E>(-(BIAS-1)))),
       is_inf(Lit(0)), // TODO: any overflow-> inf
       is_nan(Lit(0)); // TODO: NaN signal

  // The goal for normalized numbers is to get a 1 to the LSB of the n field,
  // then take the corresponding bits of the m field and put them in the output
  // m.
  node norm_lshift(msb < Lit<CLOG2(N + M)>(M)); // Shift left if MSB in m(not n)
  bvec<CLOG2(N + M)> shiftamt;

  // If we're subnormal, we are moving a 0 instead of the msb. This 0 lives at
  // bit M - (BIAS-1) - e, which is usually out of range. If it's not, we set
  // shamt accordingly using this value instead of msb.
  bvec<CLOG2(N + M)> subnorm_msb(
    Lit<CLOG2(N + M)>(M - (BIAS-1)) - Zext<CLOG2(N + M)>(_(in, "e"))
  );
  node subnorm_out_of_range(
    si<E>(_(in, "e")) > si<E>(Lit<E>(M - (BIAS-1)))
  ), subnorm_lshift(subnorm_msb < Lit<CLOG2(N + M)>(M));

  // Shift by distance from ideal position.
  node lshift(subnorm && subnorm_lshift || !subnorm && norm_lshift);
  Cassign(shiftamt).
    // Keep significand of NaNs by keeping them aligned to the binary point.
    IF(is_inf || is_nan, Lit<CLOG2(N + M)>(0)).
    IF(subnorm).
      IF(subnorm_lshift, Lit<CLOG2(N + M)>(M) - subnorm_msb).
      ELSE(subnorm_msb - Lit<CLOG2(N + M)>(M)).
    END().ELSE().
      IF(norm_lshift, Lit<CLOG2(N + M)>(M) - msb).
      ELSE(msb - Lit<CLOG2(N + M)>(M)).
    END();
  bvec<N + M> norm_m(Shifter(nm, shiftamt, Lit(0), Lit(0), !lshift));

  fp<EE, MM> r;

  _(r, "s") = _(in, "s");

  Cassign(_(r, "e")).
    IF(subnorm, Lit<EE>(0)).
    IF(is_nan || is_inf, ~Lit<EE>(0)).
    ELSE(Zext<EE>(norm_e + Lit<E>(BIAS)));

  Cassign(_(r, "m")).
    IF(zero || (subnorm && subnorm_out_of_range), Lit<MM>(0)).
    ELSE(Reverse(Zext<MM>(Reverse(norm_m[range<0,M-1>()]))));

  return r;
}

template <unsigned E, unsigned M, unsigned N> fp<E, M> IToF(const bvec<N> &x) {
  ifp<E, N, M> f;
  _(f, "s") = Lit(0);
  _(f, "e") = Lit<E>(0);
  _(f, "n") = x;
  _(f, "m") = Lit<M>(0);
  return IfpToFp<E, M>(f);
}

template <unsigned N, unsigned E, unsigned M> bvec<N> FToI(const fp<E, M> &x) {
  // If our exponent (minus bias) is less than zero, our integer value is zero.
  const unsigned BIAS((1ull<<(E-1))-1);

  node zero(_(x, "e") < Lit<E>(BIAS)),
       lshift(Zext<32>(_(x, "e")) > Lit<32>(BIAS + M));

  bvec<CLOG2(N+M)> shamt;
  Cassign(shamt).
    IF(lshift, Zext<CLOG2(N+M)>(_(x, "e") - Lit<E>(M + BIAS))).
    ELSE(Zext<CLOG2(N+M)>(Lit<E>(M + BIAS) - _(x, "e")));
  
  bvec<N+M> val(
    Shifter(Zext<N+M>(Cat(Lit(1), _(x,"m"))), shamt, Lit(0), Lit(0), !lshift)
  );

  bvec<N> n(Zext<N>(val));

  return Mux(zero, Mux(_(x, "s"), n, -n), Lit<N>(0));
}

template <unsigned E, unsigned M>
  node operator<(const fp<E, M> &a, const fp<E, M> &b)
{
  node aneg(_(a, "s")), bneg(_(b, "s"));

  // These are the magnitudes in the pretend-sign-magnitude numbers we use for
  // comparison.
  bvec<E + M> a_em(Cat(_(a, "e"), _(a, "m"))), b_em(Cat(_(b, "e"), _(b, "m")));

  return (aneg && !bneg) ||
         (aneg && bneg && b_em < a_em) ||
         (!aneg && !bneg && a_em < b_em);
}

template <unsigned E, unsigned M>
  node operator>(const fp<E, M> &a, const fp<E, M> &b) { return b < a; }

template <unsigned E, unsigned M>
  node operator<=(const fp<E, M> &a, const fp<E, M> &b) { return !(a > b); }

template <unsigned E, unsigned M>
  node operator>=(const fp<E, M> &a, const fp<E, M> &b) { return !(a < b); }

// Sometimes we only care if the magnitude is greater.
template <unsigned E, unsigned M>
  node Bigger(const fp<E, M> &a, const fp<E, M> &b)
{
  bvec<E + M> a_em(Cat(_(a, "e"), _(a, "m"))), b_em(Cat(_(b, "e"), _(b, "m")));
  return a_em > b_em;
}

template <unsigned E, unsigned M> fp<E, M> operator-(const fp<E, M> &x) {
  fp<E, M> r;
  _(r, "s") = !_(x, "s");
  _(r, "e") = _(x, "e");
  _(r, "m") = _(x, "m");
  return r;
}

template <unsigned E, unsigned M>
  fp<E, M> operator+(const fp<E, M> &a, const fp<E, M> &b)
{
  // Is a or b bigger? (larger magnitude)
  node a_bigger(Bigger(a, b));
  fp<E, M> big(Mux(a_bigger, b, a)), sml(Mux(a_bigger, a, b));
 
  // Convert to internal representation
  ifp<E+1, 2, M> ai(FpToIfp<E+1, 2, M>(big)), bi(FpToIfp<E+1, 2, M>(sml));
  
  // We can assume we're normalized or e=-(BIAS-1), so we can use the difference
  // of the exponents to determine the shift amount.
  bvec<E + CLOG2(2 + M)> shamt(Sext<E + CLOG2(2 + M)>(_(ai, "e")) -
                               Sext<E + CLOG2(2 + M)>(_(bi, "e")));
  node do_nothing(shamt > Lit<E + CLOG2(2 + M)>(M));

  bvec<2 + M> a_nm(Cat(_(ai, "n"), _(ai, "m"))),
              b_nm(Cat(_(bi, "n"), _(bi, "m"))),
              bsh_nm(b_nm >> Zext<CLOG2(2 + M)>(shamt)), sum_nm;

  Cassign(sum_nm).
    IF(do_nothing, a_nm).
    IF(_(ai, "s") == _(bi, "s"), a_nm + bsh_nm).
    ELSE(a_nm - bsh_nm);

  ifp<E + 1, 2, M> ri;
  _(ri, "s") = _(ai, "s");
  _(ri, "e") = _(ai, "e");
  Cat(_(ri, "n"), _(ri, "m")) = sum_nm;

  return IfpToFp<E, M>(ri);
}

template <unsigned E, unsigned M>
  fp<E, M> operator-(const fp<E, M> &a, const fp<E, M> &b)
{
  return a + -b;
}

template <unsigned E, unsigned M>
  fp<E, M> operator*(const fp<E, M> &a, const fp<E, M> &b)
{
  ifp<E+1, 2, M> ai(FpToIfp<E+1, 2, M>(a)), bi(FpToIfp<E+1, 2, M>(b));

  ifp<E+1, 2, M> ri;

  bvec<2 + M> a_nm(Cat(_(ai, "n"), _(ai, "m"))),
              b_nm(Cat(_(bi, "n"), _(bi, "m"))),
              prod_nm;

  prod_nm = (Zext<4 + 2*M>(a_nm) * Zext<4 + 2*M>(b_nm))[range<M, 2*M + 1>()];

  _(ri, "s") = _(ai, "s") != _(bi, "s");
  _(ri, "e") = _(ai, "e") + _(bi, "e");
  Cat(_(ri, "n"), _(ri, "m")) = prod_nm;

  return IfpToFp<E, M>(ri);
}

}

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

  tap("increment", LitCpx<T>(inc, 0));
  TAP(next_x);
  TAP(next_x0);
}

void build() {
  typedef fp32 T;
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
