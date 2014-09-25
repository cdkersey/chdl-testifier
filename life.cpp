#include <fstream>

#include <cstdlib>

#include <chdl/chdl.h>
#include <chdl/cassign.h>
#include <chdl/egress.h>
#include <chdl/reset.h>

#include "testify-module.h"

TESTIFY_VERSION("2014-09-24");

using namespace std;
using namespace chdl;

template<unsigned N> bvec<CLOG2(N+1)> PopCount(bvec<N> x) {
  return Zext<CLOG2(N+1)>(PopCount(x[range<0,N/2-1>()])) +
         Zext<CLOG2(N+1)>(PopCount(x[range<N/2,N-1>()]));
}

template<> bvec<1> PopCount<1>(bvec<1> x) { return x; }

node LifeCell(bvec<8> neighbors, bool init = false) {
  bvec<4> count(PopCount(neighbors));
  node next_alive, alive(Reg(next_alive, init));

  Cassign(next_alive).
    IF(count < Lit<4>(2), Lit(0)).
    IF(count > Lit<4>(3), Lit(0)).
    IF(count == Lit<4>(3), Lit(1)).
    ELSE(alive);

  return alive;
} 

template <bool T, unsigned X, unsigned Y>
  node Get(vec<Y, bvec<X> > &g, int i, int j)
{
  if (T) {
    while (i < 0) i += X;
    while (j < 0) j += Y;
    while (i >= X) i -= X;
    while (j >= Y) j -= Y;
  }

  if (i < 0 || i >= X || j < 0 || j >= Y) return Lit(0);
  else return g[i][j];
}

template <bool T, unsigned X, unsigned Y>
  bvec<8> Neighbors(vec<Y, bvec<X> > &g, unsigned i, unsigned j)
{
  auto G(Get<T, X, Y>);

  return bvec<8>{
    G(g, i-1, j-1), G(g, i-1,   j), G(g, i-1, j+1), G(g,   i, j-1),
    G(g,   i, j+1), G(g, i+1, j-1), G(g, i+1,   j), G(g, i+1, j+1)
  };
}

template <bool T, unsigned X, unsigned Y>
  vec<Y, bvec<X> > LifeGrid(bool *init)
{
  vec<Y, bvec<X> > g;

  for (unsigned i = 0; i < Y; ++i)
    for (unsigned j = 0; j < X; ++j)
      g[i][j] = LifeCell(Neighbors<T>(g, i, j), init[j * X + i]);

  return g;
}

void build() {
  const unsigned X(64), Y(64);
  bool init[X*Y];

  srand(100);
  for (unsigned i = 0; i < X*Y; ++i) init[i] = (rand()%4 == 0);

  vec<Y, bvec<X> > g = LifeGrid<1, X, Y>(init);

  TAP(g);
}
