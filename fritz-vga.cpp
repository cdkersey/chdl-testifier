#include <chdl/chdl.h>
#include <chdl/ag.h>

#include "testify-module.h"

TESTIFY_VERSION("2014-11-13");

// Video signal with arbitrary pixel type
template <typename T> using vsig = chdl::ag<STP("hsync"), chdl::node,
                                   chdl::ag<STP("vsync"), chdl::node,
                                   chdl::ag<STP("vid"), T> > >;
vsig<chdl::bvec<3> > Testgen();

using namespace std;
using namespace chdl;

#define VGA

// Video timings, units of lines (vertical) or clock cycles (horizontal)
// NTSC: Assuming 24MHz clock
struct vtiming_ntsc {
  static const unsigned vfp, vtip, vbp, lines, hfp, htip, hbp, linedur;
};

const unsigned vtiming_ntsc::vfp(0), vtiming_ntsc::vtip(3),
               vtiming_ntsc::vbp(19),
               vtiming_ntsc::lines(240), vtiming_ntsc::hfp(36),
               vtiming_ntsc::htip(104), vtiming_ntsc::hbp(104),
               vtiming_ntsc::linedur(1280);

// VGA: Assuming 50MHz clock
struct vtiming_vga {
  static const unsigned vfp, vtip, vbp, lines, hfp, htip, hbp, linedur; 
};

const unsigned vtiming_vga::vfp(10), vtiming_vga::vtip(2), vtiming_vga::vbp(33),
               vtiming_vga::lines(480), vtiming_vga::hfp(32),
               vtiming_vga::htip(192), vtiming_vga::hbp(96),
               vtiming_vga::linedur(1280);

bvec<4> ToComposite(vsig<bvec<3> > &v) {
  bvec<4> c(Mux(_(v, "vsync") != _(v, "hsync"),
            Lit<4>(5) + Zext<4>(_(v, "vid")), Lit<4>(0)));
  return c;
}

typedef ag<STP("hsync"), node,
        ag<STP("vsync"), node,
        ag<STP("dacdata"), vec<3, bvec<4> > > > > vgasig;

vgasig ToVga(const vsig<bvec<3> > &in) {
  vgasig out;
  _(out, "hsync") = _(in, "hsync");
  _(out, "vsync") = _(in, "vsync");
}

template <unsigned N> bvec<N> CountTo(unsigned max, node en, node &atmax) {
  bvec<N> c, next_c;

  if (max == (1ul << N) - 1) {
    next_c = c + Lit<N>(1);
  } else {
    next_c = Mux(c == Lit<N>(max), c + Lit<N>(1), Lit<N>(0));
  }

  c = Wreg(en, next_c);

  atmax = c == Lit<N>(max);

  return c;
}

// Generates a 320x240x3 test pattern based on the image in fritz.hex
template <typename V> vsig<bvec<3> > testpattern() {
  // 320x240x3 graphics: 28800 bits, less than 32kB

  const unsigned TOT_LINES(V::vfp + V::vtip + V::vbp + V::lines),
                 TOT_COLS(V::hfp + V::htip + V::linedur),
                 VBLANK_LINES(V::vfp + V::vtip + V::vbp),
                 HBLANK_COLS(V::hfp + V::htip + V::hbp),
                 LINE_BITS(CLOG2(TOT_LINES)),
                 COL_BITS(CLOG2(TOT_COLS));

  node col_end, frame_end;
  bvec<COL_BITS> colct(CountTo<COL_BITS>(TOT_COLS, Lit(1), col_end));
  bvec<LINE_BITS> linect(CountTo<LINE_BITS>(TOT_LINES, col_end, frame_end));

  TAP(colct);
  TAP(linect);

  bvec<17> img_addr((Zext<17>(linect[range<1,LINE_BITS-1>()])
                     - Lit<17>(VBLANK_LINES/2))*Lit<17>(320)
                   + Zext<17>(colct[range<2,COL_BITS-1>()]) 
                     - Lit<17>(HBLANK_COLS/4));
  TAP(img_addr);
  bvec<3> rom_data(LLRom<17, 3>(img_addr, "fritz.hex"));
  TAP(rom_data);

  vsig<bvec<3> > out;
  _(out, "vsync") = linect >= Lit<LINE_BITS>(V::vfp) &&
                    linect < Lit<LINE_BITS>(V::vfp + V::vtip);
#ifdef NTSC
  _(out, "hsync") = colct >= Lit<COL_BITS>(V::hfp) &&
                    colct < Lit<COL_BITS>(V::hfp + V::htip)
                || linect < Lit<LINE_BITS>(V::vtip * 2) &&
                   colct >= Lit<COL_BITS>(TOT_COLS/2 + V::hfp) &&
                   colct < Lit<COL_BITS>(TOT_COLS/2 + V::hfp + V::htip);
#else
  _(out, "hsync") = colct >= Lit<COL_BITS>(V::hfp) &&
                    colct < Lit<COL_BITS>(V::hfp + V::htip);
#endif

  _(out, "vid") = rom_data & bvec<3>(linect > Lit<LINE_BITS>(VBLANK_LINES) &&
                  colct > Lit<COL_BITS>(HBLANK_COLS));

  TAP(out);

  return out;
}

vsig<bvec<3> > Testgen() {
  return testpattern<vtiming_vga>();
}

void build() {
  vsig<bvec<3> > out(Testgen());
  TAP(out);
}
