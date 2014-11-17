// We would like to be able to build true NoCs from our bucket of parts.
#include <sstream>
#include <fstream>

#include <chdl/chdl.h>
#include <chdl/ag.h>
#include <chdl/cassign.h>
#include <chdl/net.h>
#include <chdl/lfsr.h>

#include "testify-module.h"

using namespace chdl;
using namespace std;

TESTIFY_VERSION("2014-09-23-PRE_RELEASE");

// 1-way Link with credit-based flow control, with 2^B-entry buffers on the
// receiving end.
template <typename T, unsigned B> using cflit =
  ag<STP("credit"), node,
  ag<STP("valid"), node,
  ag<STP("contents"), T > > >;

// Simple flit type for xy routing. 
template <unsigned N, typename T> using xy =
  ag<STP("x"), bvec<N>,
  ag<STP("y"), bvec<N>,
  ag<STP("data"), T> > >;

// Adapt ready/valid flow control to credit-based flow control.
template <typename T, unsigned B>
  void CBuffer(flit<T> &out, cflit<T, B> &in)
{
  HIERARCHY_ENTER();

  flit<T> in_flit;
  _(in_flit, "valid") = _(in, "valid");
  _(in_flit, "contents") = _(in, "contents");

  _(in, "credit") = _(out, "ready") && _(out, "valid");

  Buffer<B>(out, in_flit);

  HIERARCHY_EXIT();
}

template <typename T, unsigned B>
  void CSend(cflit<T, B> &out, flit<T> &in)
{
  HIERARCHY_ENTER();

  // The credit counter. 
  bvec<B + 1> next_c, c(Reg(next_c, (1<<B)));
  _(in, "ready") = (c != Lit<B + 1>(1<<B));
  Cassign(next_c).
    IF(_(out, "credit"), c + Lit<B + 1>(1)).
    IF(_(in, "ready") && _(in, "valid"), c - Lit<B + 1>(1)).
    ELSE(c);

  _(out, "contents") = _(in, "contents");
  _(out, "valid") = _(in, "valid");

  HIERARCHY_EXIT();
}

// Our NOC router.
template <typename T, unsigned B, unsigned N, typename F>
  void NocRouter(flit<T> &out, flit<T> &in,
                 vec<N, cflit<T, B> > &send, vec<N, cflit<T, B> > &rec,
                 F Func)
{
  HIERARCHY_ENTER();

  // The input buffers and ingress port
  vec<N + 1, flit<T> > rec_flit;
  for (unsigned i = 0; i < N; ++i)
    CBuffer<T, B>(rec_flit[i], rec[i]);
  rec_flit[N] = in;

  // The output controllers and egress port
  vec<N + 1, flit<T> > send_flit;
  for (unsigned i = 0; i < N; ++i)  // TODO: FIGURE OUT AND UNCOMMENT
    CSend<T, B>(send[i], send_flit[i]);
  send_flit[N] = out;

  // The routing function itself is the switch fabric.
  Func(send_flit, rec_flit);

  HIERARCHY_EXIT();
}

// The routing function used for standard mesh x/y routing. Order is clockwise
// E N W S, local.
template <unsigned N, typename T>
  void FuncXY(vec<5, flit<xy<N, T> > > &out, vec<5, flit<xy<N, T> > > &in)
{
  vec<5, xy<N, T> > in_inc;
  vec<5, bvec<5> > sel_mat, ready_mat, ready_mat_t;
  vec<5, bvec<3> > sel;

  for (unsigned i = 0; i < 5; ++i) {
    cout << "  funcxy " << i << endl;
    xy<N, T> f(_(in[i], "contents")), g(in_inc[i]);

    _(g, "data") = _(f, "data");

    node v(_(in[i], "valid")),
         l(v && _(f, "x") == Lit<N>(0) && _(f, "y") == Lit<N>(0)),
         e(v && !_(f, "x")[N-1]),
         w(v && _(f, "x")[N-1]),
         n(v && !e && !w && _(f, "y")[N-1]),
         s(v && !e && !w && !_(f, "y")[N-1]);

    sel_mat[0][i] = e;
    sel_mat[1][i] = n;
    sel_mat[2][i] = w;
    sel_mat[3][i] = s;
    sel_mat[4][i] = l;

    Cassign(_(g, "x")).
      IF(w, _(f, "x") - Lit<N>(1)).
      IF(e, _(f, "x") + Lit<N>(1)).
      ELSE(_(f, "x"));

    Cassign(_(g, "y")).
      IF(n, _(f, "y") - Lit<N>(1)).
      IF(s, _(f, "y") + Lit<N>(1)).
      ELSE(_(f, "y"));
  }

  // Not the best arbitration, but easy-to-implement.
  for (unsigned i = 0; i < 5; ++i) {
    sel[i] = Log2(sel_mat[i]);
    ready_mat[i] = Lit<5>(1) << sel[i];
    _(out[i], "valid") = OrN(sel_mat[i]);
    _(out[i], "contents") = Mux(sel[i], in_inc);
  }

  // Transpose the ready matrix
  for (unsigned i = 0; i < 5; ++i)
    for (unsigned j = 0; j < 5; ++j)
      ready_mat_t[i][j] = ready_mat[j][i];

  // The input ready signals
  for (unsigned i = 0; i < 5; ++i)
    _(in[i], "ready") = OrN(ready_mat_t[i]);
}

// Generate random traffic.
template <unsigned N, typename T> void TrafficGen(flit<xy<N, T> > &out) {
  node en(/*_(out, "ready")*/Lit(1));

  _(out, "valid") = AndN(Lfsr<2, 31, 7>(en, rand()));

  _(_(out, "contents"), "x") = Lfsr<N, 63, 3>(en, rand());
  _(_(out, "contents"), "y") = Lfsr<N, 63, 1>(en, rand());
}

template <unsigned N, unsigned B, typename T> void Mesh() {
  // Number of bits in a row/col address
  const unsigned NN(CLOG2(N)+1);

  typedef xy<NN, T> xy_t;
  typedef flit<xy_t> flit_t;
  typedef cflit<xy_t, B> cflit_t; 

  vec<N, vec<N, cflit_t> > net0, net1, net2, net3;
  TAP(net0);
  TAP(net3);

  for (unsigned i = 0; i < N; ++i) {
    for (unsigned j = 0; j < N; ++j) {
      cout << "Building " << i << ", " << j << endl;

      // Add a 3-cycle latency to all network traffic. On a fast NoC, these
      // registers would presumably be physically distributed along the line as
      //  six latches.
      _(net1[i][j], "contents") = Reg(_(net0[i][j], "contents"));
      _(net2[i][j], "contents") = Reg(_(net1[i][j], "contents"));
      _(net3[i][j], "contents") = Reg(_(net2[i][j], "contents"));
      _(net1[i][j], "valid") = Reg(_(net0[i][j], "valid"));
      _(net2[i][j], "valid") = Reg(_(net1[i][j], "valid"));
      _(net3[i][j], "valid") = Reg(_(net2[i][j], "valid"));
      _(net0[i][j], "credit") = Reg(_(net1[i][j], "credit"));
      _(net1[i][j], "credit") = Reg(_(net2[i][j], "credit"));
      _(net2[i][j], "credit") = Reg(_(net3[i][j], "credit"));

      // Construct the input and output vectors for this tile.
      vec<4, cflit_t> out, in;
      if (i != N-1) { in[0] = net3[i+1][j]; out[0] = net0[i+1][j]; }
      if (j != 0)   { in[1] = net3[i][j-1]; out[1] = net0[i][j-1]; }
      if (i != 0)   { in[2] = net3[i-1][j]; out[2] = net0[i-1][j]; }
      if (j != N-1) { in[3] = net3[i][j+1]; out[3] = net0[i][j+1]; }

      flit_t egress, ingress;
      NocRouter<xy_t, B>(egress, ingress, out, in, FuncXY<NN, T>);

      ostringstream oss; oss << '_' << i << '_' << j;
      tap(string("net_in") + oss.str(), in); 
      tap(string("net_out") + oss.str(), out); 
      tap(string("net_egress") + oss.str(), egress); 
      tap(string("net_ingress") + oss.str(), ingress); 

      TrafficGen(ingress);
    }
  }
}

void build() {
  const unsigned FLIT_SZ(128), NET_SZ(2), BUF_SZ(4);
  Mesh<NET_SZ, BUF_SZ, bvec<FLIT_SZ>>(); 
}
