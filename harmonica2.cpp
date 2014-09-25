#include <chdl/chdl.h>
#include <chdl/ag.h>
#include <chdl/net.h>
#include <chdl/ingress.h>
#include <chdl/egress.h>
#include <chdl/counter.h>

#include <fstream>

#include "testify-module.h"

TESTIFY_VERSION("2014-09-24");

#ifndef HARMONICA_CONFIG_H
#define HARMONICA_CONFIG_H

const unsigned W(16), // Total warps
               L(4), // Number of SIMD lanes
               N(32), // Number of bits in a machine word
               R(8), // Number of GPRs/predicate registers
               LINE(16), // Words per cache line
               ROMSZ(4096), // Instruction ROM size
               IPDOM_STACK_SZ(2), // Hardware stack used for control flow.
               DUMMYCACHE_SZ(64); // Cache lines in main memory

// Doubled constants mean log base 2 of the corresponding constant. LL bits can
// hold a lane ID, RR bits can uniquely identify a register.
const unsigned WW(chdl::CLOG2(W)), RR(chdl::CLOG2(R)), LL(chdl::CLOG2(L)),
               NN(chdl::CLOG2(N));

const bool SRAM_REGS(false), // Use SRAM for register files
           FPGA(false), // Produce Verilog netlist for FPGA
           FPGA_IO(false), // Console output from DummyCache
           NETLIST(false), // Produce a .netl file
           SIMULATE(true), // Run a simulation
           SOFT_IO(true),  // Software I/O. Console output to stdout
           DEBUG_MEM(false); // Print all memory operations to stderr

#endif
#ifndef HARMONICA_HARPINST_H
#define HARMONICA_HARPINST_H

using namespace chdl; // TODO: this is downright naughty

enum argclass {
  AC_NONE, AC_1REG, AC_2REG, AC_3REG, AC_3REGSRC, AC_1IMM, AC_2IMM, AC_3IMM,
  AC_3IMMSRC, AC_PREG_REG, AC_2PREG, AC_3PREG
};

enum argclass_bit {
  ACBIT_NONE     = 0x001,
  ACBIT_1REG     = 0x002,
  ACBIT_2REG     = 0x004,
  ACBIT_3REG     = 0x008,
  ACBIT_3REGSRC  = 0x010,
  ACBIT_1IMM     = 0x020,
  ACBIT_2IMM     = 0x040,
  ACBIT_3IMM     = 0x080,
  ACBIT_3IMMSRC  = 0x100,
  ACBIT_PREG_REG = 0x200,
  ACBIT_2PREG    = 0x400,
  ACBIT_3PREG    = 0x800
};

// This structure defines an instruction type and provides a way to decode it.
// Issues:
//   Assumes, when reading predicate register fields, that R == P.
template <unsigned N, unsigned R, unsigned P> struct harpinst {
  harpinst(bvec<N> inst): inst(inst), argclass(argclass_hw()) {}

  node    has_pred()  { return inst[N-1]; }
  bvec<P> get_pred()  { return inst[range<N-1-P, N-2>()]; }
  bvec<6> get_opcode(){ return inst[range<N-1-P-6, N-2-P>()]; }
  bvec<4> get_argclass() { return Enc(argclass); }

  node    has_pdst()  {
    return argclass[AC_PREG_REG] || argclass[AC_2PREG] || argclass[AC_3PREG];
  }
  bvec<P> get_pdst()  { return inst[range<N-1-P-6-R, N-1-P-7>()]; }

  node    has_psrc0() {
    return argclass[AC_2PREG] || argclass[AC_3PREG];
  }
  bvec<P> get_psrc0() { return inst[range<N-1-P-6-2*R, N-1-P-7-R>()]; }

  node    has_psrc1() { return argclass[AC_3PREG]; }
  bvec<P> get_psrc1() { return inst[range<N-1-P-6-3*R, N-1-P-7-2*R>()]; }

  node    has_rdst()  {
    return argclass[AC_2REG] || argclass[AC_3REG]
        || argclass[AC_2IMM] || argclass[AC_3IMM]
        || is_jal();
  }

  bvec<R> get_rdst()  { return inst[range<N-1-P-6-R, N-1-P-7>()]; }

  node    has_rsrc0() {
    return argclass[AC_1REG] || argclass[AC_2REG]
        || argclass[AC_3REG] || argclass[AC_3REGSRC]
        || argclass[AC_3IMM] || argclass[AC_3IMMSRC]
        || argclass[AC_PREG_REG];
  }
  bvec<R> get_rsrc0() {
    return Mux(argclass[AC_1REG] ||
               argclass[AC_3REGSRC] || argclass[AC_3IMMSRC],
                 inst[range<N-1-P-6-2*R, N-1-P-R-7>()], 
                 inst[range<N-1-P-6-R, N-1-P-7>()]);
  }

  node    has_rsrc1() {
    return argclass[AC_3REG] || argclass[AC_3REGSRC] || argclass[AC_3IMMSRC];
  }
  bvec<R> get_rsrc1() { return Mux(argclass[AC_3REGSRC] || argclass[AC_3IMMSRC],
                                     inst[range<N-1-P-6-3*R, N-1-P-2*R-7>()],
                                     inst[range<N-1-P-6-2*R, N-1-P-R-7>()]);
  }

  node    has_rsrc2() { return argclass[AC_3REGSRC]; }
  bvec<R> get_rsrc2() { return inst[range<N-1-P-6-3*R, N-1-P-2*R-7>()]; }

  node    has_imm() {
    return argclass[AC_1IMM] || argclass[AC_2IMM] || argclass[AC_3IMM]
        || argclass[AC_3IMMSRC];
  }
  bvec<N> get_imm() {
    bvec<N> imm_huge(Sext<N>(inst[range<0, N-1-P-7>()])),
            imm_large(Sext<N>(inst[range<0, N-1-P-7-R>()])),
            imm_small(Sext<N>(inst[range<0, N-1-P-7-2*R>()]));

    return Mux(argclass[AC_1IMM],
                 Mux(argclass[AC_2IMM], imm_small, imm_large),
                 imm_huge);
  }

  node is_jal() { return get_opcode() == Lit<6>(0x20)
                      || get_opcode() == Lit<6>(0x21)
                      || get_opcode() == Lit<6>(0x1b)
                      || get_opcode() == Lit<6>(0x1c); }

  node is_jmp() { return get_opcode() == Lit<6>(0x1b)
                      || get_opcode() == Lit<6>(0x1c)
                      || get_opcode() == Lit<6>(0x1d)
                      || get_opcode() == Lit<6>(0x1e)
                      || get_opcode() == Lit<6>(0x20)
                      || get_opcode() == Lit<6>(0x21)
                      || get_opcode() == Lit<6>(0x22)
                      || get_opcode() == Lit<6>(0x2f); }
  node is_store() { return get_opcode() == Lit<6>(0x24); }
  node is_halt() { return get_opcode() == Lit<6>(0x2d); }
  node is_clone() { return get_opcode() == Lit<6>(0x1f); }
  node is_wclone() { return get_opcode() == Lit<6>(0x3a); }
  node is_skep() { return get_opcode() == Lit<6>(0x30); }

  node is_spawn() { return get_opcode() == Lit<6>(0x20)
                        || get_opcode() == Lit<6>(0x21); }
  node is_term() { return get_opcode() == Lit<6>(0x22); } // jmprt
  node is_split() { return get_opcode() == Lit<6>(0x3a); } // split
  node is_join() { return get_opcode() == Lit<6>(0x3b); } // join

  node is_prot() { return get_opcode() == Lit<6>(0x30)    // skep
                       || get_opcode() == Lit<6>(0x01)    // di 
                       || get_opcode() == Lit<6>(0x02)    // ei
                       || get_opcode() == Lit<6>(0x03)    // tlbadd
                       || get_opcode() == Lit<6>(0x04)    // tlbflush
                       || get_opcode() == Lit<6>(0x32)    // tlbrm
                       || get_opcode() == Lit<6>(0x2f)    // jmpru
                       || get_opcode() == Lit<6>(0x31)    // reti
                       || get_opcode() == Lit<6>(0x2d); } // halt

  node is_ldst() { return get_opcode() == Lit<6>(0x23)
                       || get_opcode() == Lit<6>(0x24); }

  // Returns a bit vector representing the argument class of this instruction.
  bvec<12> argclass_hw() {
    HIERARCHY_ENTER();
    bvec<6> op(get_opcode());
    bvec<12> ac;

    ac[AC_NONE] = op == Lit<6>(0x2d);
    ac[AC_1REG] = op == Lit<6>(0x1e) || op == Lit<6>(0x1f)
               || op == Lit<6>(0x22) || op == Lit<6>(0x2f)
               || op == Lit<6>(0x30) || op == Lit<6>(0x32);
    ac[AC_2REG] = op == Lit<6>(0x05) || op == Lit<6>(0x06)
               || op == Lit<6>(0x1c) || op == Lit<6>(0x33)
               || op == Lit<6>(0x34) || op == Lit<6>(0x39);
    ac[AC_3REG] = op == Lit<6>(0x07) || op == Lit<6>(0x08)
               || op == Lit<6>(0x09) || op == Lit<6>(0x0a)
               || op == Lit<6>(0x0b) || op == Lit<6>(0x0c)
               || op == Lit<6>(0x0d) || op == Lit<6>(0x0e)
               || op == Lit<6>(0x0f) || op == Lit<6>(0x10)
               || op == Lit<6>(0x35) || op == Lit<6>(0x36)
               || op == Lit<6>(0x37);
    ac[AC_3REGSRC] = Lit(0);
    ac[AC_1IMM] = op == Lit<6>(0x1d);
    ac[AC_2IMM] = op == Lit<6>(0x1b) || op == Lit<6>(0x25);
    ac[AC_3IMM] = op == Lit<6>(0x11) || op == Lit<6>(0x12)
               || op == Lit<6>(0x13) || op == Lit<6>(0x14)
               || op == Lit<6>(0x15) || op == Lit<6>(0x16)
               || op == Lit<6>(0x17) || op == Lit<6>(0x18)
               || op == Lit<6>(0x19) || op == Lit<6>(0x1a)
               || op == Lit<6>(0x23);
    ac[AC_3IMMSRC] = op == Lit<6>(0x24);
    ac[AC_PREG_REG] = op == Lit<6>(0x26) || op == Lit<6>(0x2b)
                   || op == Lit<6>(0x2c);
    ac[AC_2PREG] = op == Lit<6>(0x2a);
    ac[AC_3PREG] = op == Lit<6>(0x27) || op == Lit<6>(0x28)
                || op == Lit<6>(0x29);

    HIERARCHY_EXIT();

    return ac;  
  }

  bvec<N> inst;
  bvec<12> argclass;
};

#endif
#ifndef HARMONICA_INTERFACES_H
#define HARMONICA_INTERFACES_H

namespace chdl {
  enum Tstate {
    TS_USER, TS_DIVERGENT_BR, TS_PENDING_INT, TS_KERNEL, N_TSTATES
  };

  const unsigned SS(CLOG2(N_TSTATES));

  // Basic warp variables
  typedef ag<STP("state"), bvec<SS>,
          ag<STP("pc"), bvec<N>,
          ag<STP("active"), bvec<L>, 
          ag<STP("id"), bvec<WW> > > > > warp_t;

  // Predicate read values
  typedef ag<STP("pmask"), bvec<L>,
          ag<STP("val0"), bvec<L>,
          ag<STP("val1"), bvec<L> > > > pval_t;

  // Writeback predicate values
  typedef ag<STP("mask"), bvec<L>,
          ag<STP("val"), bvec<L>,
          ag<STP("wid"), bvec<WW>, 
          ag<STP("dest"), bvec<RR> > > > > pwb_t;

  // Register read values
  typedef ag<STP("val0"), vec<L, bvec<N> >,
          ag<STP("val1"), vec<L, bvec<N> >,
          ag<STP("val2"), vec<L, bvec<N> > > > > rval_t;

  // Writeback register values
  typedef ag<STP("mask"), bvec<L>,
          ag<STP("val"), vec<L, bvec<N> >,
          ag<STP("wid"), bvec<WW>, 
          ag<STP("dest"), bvec<RR>,
          ag<STP("clone"), node,
          ag<STP("clonesrc"), bvec<LL>,
          ag<STP("clonedest"), bvec<LL> > > > > > > > rwb_t;

  // Sched->Fetch
  typedef flit<warp_t> sched_fetch_t;

  // Fetch->Predicate
  typedef flit<ag<STP("warp"), warp_t,
               ag<STP("ir"), bvec<N> > > > fetch_pred_t;

  // Predicate->Reg
  typedef flit<ag<STP("warp"), warp_t,
               ag<STP("ir"),   bvec<N>,
               ag<STP("pval"), pval_t> > > > pred_reg_t;

  // Reg->Dispatch, Dispatch->FU
  typedef ag<STP("warp"), warp_t,
          ag<STP("ir"), bvec<N>,
          ag<STP("pval"), pval_t,
          ag<STP("rval"), rval_t> > > > reg_func_int_t;

  typedef flit<reg_func_int_t> reg_func_t;

  // FU->Arbiter, Arbiter->Splitter
  typedef flit<ag<STP("warp"), warp_t,
               ag<STP("spawn"), node,
               ag<STP("spawn_pc"), bvec<N>,
               ag<STP("rwb"), rwb_t,
               ag<STP("pwb"), pwb_t> > > > > > func_splitter_t;

  // Splitter->Register
  typedef flit<rwb_t> splitter_reg_t;

  // Splitter->Predicate
  typedef flit<pwb_t> splitter_pred_t;

  // Splitter->Sched
  typedef flit<ag<STP("warp"), warp_t, 
               ag<STP("spawn"), node,
               ag<STP("spawn_pc"), bvec<N> > > > > splitter_sched_t;

  // Memory Request
  typedef flit<ag<STP("warp"), warp_t,
               ag<STP("wr"), node,
               ag<STP("mask"), bvec<L>,
               ag<STP("a"), vec<L, bvec<N> >,
               ag<STP("d"), vec<L, bvec<N> > > > > > > > mem_req_t;    

  // Cache Request
  typedef ag<STP("warp"), warp_t,
          ag<STP("lane"), bvec<LL>,
          ag<STP("wr"), node,
          ag<STP("mask"), bvec<LINE>,
          ag<STP("a"), bvec<N>,
          ag<STP("d"), vec<LINE, bvec<N> > > > > > > > cache_req_int_t;

  typedef flit<cache_req_int_t> cache_req_t;

  // Memory Response
  typedef flit<ag<STP("warp"), warp_t,
               ag<STP("q"), vec<L, bvec<N> > > > > mem_resp_t;

  // Cache Response
  typedef flit<ag<STP("warp"), warp_t,
               ag<STP("lane"), bvec<LL>,
               ag<STP("q"), vec<LINE, bvec<N> > > > > > cache_resp_t;
}

#endif

using namespace std;

// Functional Units
void Funcunit_alu(func_splitter_t &out, reg_func_t &in);
void Funcunit_plu(func_splitter_t &out, reg_func_t &in);

// Functional Unit: (Integer) Arithmetic/Logic Unit
void Funcunit_alu(func_splitter_t &out, reg_func_t &in) {
  HIERARCHY_ENTER();
  node ready(_(out, "ready")), next_full, full(Reg(next_full));
  _(in, "ready") = !full || ready;

  Cassign(next_full).
    IF(full && _(out, "ready") && !_(in, "valid"), Lit(0)).
    IF(!full && _(in, "valid"), Lit(1)).
    ELSE(full);

  bvec<L> active(_(_(_(in, "contents"), "warp"), "active"));

  harpinst<N, RR, RR> inst(_(_(in, "contents"), "ir"));

  bvec<L> pmask(_(_(_(in, "contents"), "pval"), "pmask"));


  node ldregs(_(in, "valid") && _(in, "ready"));


  _(out, "valid") = full;
  _(_(out, "contents"), "warp") = Wreg(ldregs, _(_(in, "contents"), "warp"));

  _(_(_(out, "contents"), "rwb"), "mask") =
    Wreg(ldregs, bvec<L>(inst.has_rdst()) & pmask & active);
  _(_(_(out, "contents"), "rwb"), "dest") = Wreg(ldregs, inst.get_rdst());
  _(_(_(out, "contents"), "rwb"), "wid") =
    Wreg(ldregs, _(_(_(in, "contents"), "warp"), "id"    ));

  vec<L, bvec<N> > out_val;

  for (unsigned l = 0; l < L; ++l) {
    bvec<N> rval0(_(_(_(in, "contents"), "rval"), "val0")[l]), 
            rval1(_(_(_(in, "contents"), "rval"), "val1")[l]);

    Cassign(out_val[l]).
      IF(inst.get_opcode() == Lit<6>(0x05), -rval0). // neg
      IF(inst.get_opcode() == Lit<6>(0x06), ~rval0). // not
      IF(inst.get_opcode() == Lit<6>(0x07), rval0 & rval1). // and
      IF(inst.get_opcode() == Lit<6>(0x08), rval0 | rval1). // or
      IF(inst.get_opcode() == Lit<6>(0x09), rval0 ^ rval1). // xor
      IF(inst.get_opcode() == Lit<6>(0x0a), rval0 + rval1).  // add
      IF(inst.get_opcode() == Lit<6>(0x0b), rval0 - rval1).  // sub
      IF(inst.get_opcode() == Lit<6>(0x0f), rval0<<Zext<CLOG2(N)>(rval1)).// shl
      IF(inst.get_opcode() == Lit<6>(0x10), rval0>>Zext<CLOG2(N)>(rval1)).// shr
      IF(inst.get_opcode() == Lit<6>(0x11), rval0 & inst.get_imm()). // andi
      IF(inst.get_opcode() == Lit<6>(0x12), rval0 | inst.get_imm()). // ori
      IF(inst.get_opcode() == Lit<6>(0x13), rval0 ^ inst.get_imm()). // xori
      IF(inst.get_opcode() == Lit<6>(0x14), rval0 + inst.get_imm()).  // addi
      IF(inst.get_opcode() == Lit<6>(0x15), rval0 - inst.get_imm()).  // subi
      IF(inst.get_opcode() == Lit<6>(0x19),
         rval0 << Zext<CLOG2(N)>(inst.get_imm())). // shli
      IF(inst.get_opcode() == Lit<6>(0x1a),
         rval0 >> Zext<CLOG2(N)>(inst.get_imm())). // shri
      IF(inst.get_opcode() == Lit<6>(0x25), inst.get_imm()). // ldi
      ELSE(Lit<N>(0));

    _(_(_(out, "contents"), "rwb"), "val")[l] = Wreg(ldregs, out_val[l]);
  }

  _(_(_(out, "contents"), "rwb"), "clone") = Wreg(ldregs, inst.is_clone());
  _(_(_(out, "contents"), "rwb"), "clonedest") =
    Wreg(ldregs, _(_(_(in, "contents"), "rval"), "val0")[0][range<0, LL-1>()]);

  tap("alu_full", full);
  tap("alu_out", out);
  tap("alu_in", in);

  HIERARCHY_EXIT();
}

// Predicate Logic Unit
void Funcunit_plu(func_splitter_t &out, reg_func_t &in) {
  HIERARCHY_ENTER();
  node ready(_(out, "ready")), next_full, full(Reg(next_full));
  _(in, "ready") = !full || ready;

  Cassign(next_full).
    IF(full && _(out, "ready") && !_(in, "valid"), Lit(0)).
    IF(!full && _(in, "valid"), Lit(1)).
    ELSE(full);

  bvec<L> active(_(_(_(in, "contents"), "warp"), "active"));

  harpinst<N, RR, RR> inst(_(_(in, "contents"), "ir"));

  node ldregs(_(in, "valid") && _(in, "ready"));

  _(out, "valid") = full;
  _(_(out, "contents"), "warp") = Wreg(ldregs, _(_(in, "contents"), "warp"));
  _(_(_(out, "contents"), "pwb"), "wid") =
    Wreg(ldregs, _(_(_(in, "contents"), "warp"), "id"));

  bvec<L> pmask(_(_(_(in, "contents"), "pval"), "pmask"));

  _(_(_(out, "contents"), "pwb"), "mask") =
    Wreg(ldregs, bvec<L>(inst.has_pdst()) & pmask & active);
  _(_(_(out, "contents"), "pwb"), "dest") = Wreg(ldregs, inst.get_pdst());

  bvec<L> out_val;

  for (unsigned l = 0; l < L; ++l) {
    bvec<N> rval0(_(_(_(in, "contents"), "rval"), "val0")[l]);
    node pval0(_(_(_(in, "contents"), "pval"), "val0")[l]),
         pval1(_(_(_(in, "contents"), "pval"), "val1")[l]);

    Cassign(out_val[l]).
      IF(inst.get_opcode() == Lit<6>(0x26), OrN(rval0)).     // rtop
      IF(inst.get_opcode() == Lit<6>(0x2c), !OrN(rval0)).    // iszero
      IF(inst.get_opcode() == Lit<6>(0x2b), rval0[N-1]).     // isneg
      IF(inst.get_opcode() == Lit<6>(0x27), pval0 && pval1). // andp
      IF(inst.get_opcode() == Lit<6>(0x28), pval0 || pval1). // orp
      IF(inst.get_opcode() == Lit<6>(0x29), pval0 != pval1). // xorp
      IF(inst.get_opcode() == Lit<6>(0x2a), !pval0). // notp
      ELSE(Lit('0'));
  }

  _(_(_(out, "contents"), "pwb"), "val") = Wreg(ldregs, out_val);

  tap("plu_ldregs", ldregs);
  tap("plu_full", full);
  tap("plu_out", out);
  tap("plu_in", in);

  HIERARCHY_EXIT();
}

void Funcunit_branch(func_splitter_t &out, reg_func_t &in);

typedef ag<STP("fallthrough"), node,
        ag<STP("mask"), bvec<L>,
        ag<STP("pc"), bvec<N> > > > ipdom_stack_entry_t;

template <unsigned N, typename T> T IpdomStack(
  node &full, node &empty, node push, node pop, const T& tval, const T& bval
);

// Execute branch instructions
void Funcunit_branch(func_splitter_t &out, reg_func_t &in) {
  HIERARCHY_ENTER();
  bvec<N> pc(_(_(_(in, "contents"), "warp"), "pc"));

  node ready(_(out, "ready")), next_full, full(Reg(next_full));
  _(in, "ready") = !full || ready;

  Cassign(next_full).
    IF(full && _(out, "ready") && !_(in, "valid"), Lit(0)).
    IF(!full && _(in, "valid"), Lit(1)).
    ELSE(full);

  harpinst<N, RR, RR> inst(_(_(in, "contents"), "ir"));

  node ldregs(_(in, "valid") && _(in, "ready"));

  bvec<L> active(_(_(_(in, "contents"), "warp"), "active"));

  _(out, "valid") = full;
  _(_(_(out, "contents"), "warp"), "state") =
    Wreg(ldregs, _(_(_(in, "contents"), "warp"), "state"));
  _(_(_(out, "contents"), "warp"), "id") =
    Wreg(ldregs, _(_(_(in, "contents"), "warp"), "id"));

  bvec<L> pmask(_(_(_(in, "contents"), "pval"), "pmask")),
          outMask;

  bvec<LL> dest_idx(Lsb(active & pmask));
  node taken(OrN(active & pmask));

  bvec<N> rval0(Mux(dest_idx, _(_(_(in, "contents"), "rval"), "val0"))),
          rval1(Mux(dest_idx, _(_(_(in, "contents"), "rval"), "val1")));

  bvec<CLOG2(L+1)> lanesVal(Zext<CLOG2(L+1)>(
    Mux(inst.get_opcode()[0], rval0, rval1)
  ));

  bvec<N> destVal(Mux(inst.get_opcode()==Lit<6>(0x21), rval0, rval1));

  _(_(_(out, "contents"), "warp"), "active") = Wreg(ldregs, outMask);

  _(_(_(out, "contents"), "rwb"), "mask") =
    Wreg(ldregs, pmask & active & bvec<L>(inst.has_rdst()));
  _(_(_(out, "contents"), "rwb"), "wid") =
    Wreg(ldregs, _(_(_(in, "contents"), "warp"), "id"));
  _(_(_(out, "contents"), "rwb"), "dest") = Wreg(ldregs, inst.get_rdst());
  _(_(_(out, "contents"), "rwb"), "val") = Wreg(ldregs, pc);

  // Handle split and join instructions
  node stack_full, stack_empty, push, pop;
  ipdom_stack_entry_t otherbranch, fallthrough;
  _(otherbranch, "fallthrough") = Lit(0);
  _(fallthrough, "fallthrough") = Lit(1);
  _(otherbranch, "mask") = active & ~pmask;
  _(fallthrough, "mask") = active;
  _(otherbranch, "pc") = pc;
  _(fallthrough, "pc") = Lit<N>(0);

  ipdom_stack_entry_t top(IpdomStack<IPDOM_STACK_SZ>(
    stack_full, stack_empty, push, pop, otherbranch, fallthrough
  ));

  push = inst.get_opcode() == Lit<6>(0x3b) && _(in, "valid"); // split
  pop = inst.get_opcode() == Lit<6>(0x3c) && _(in, "valid"); // join

  Cassign(outMask).
    IF(taken).
      IF(inst.get_opcode() == Lit<6>(0x20)  // jal{i, r}s
         || inst.get_opcode() == Lit<6>(0x21),
           Zext<L>((Lit<L+1>(1) << Zext<CLOG2(L+1)>(lanesVal)) - Lit<L+1>(1))).
      IF(inst.get_opcode() == Lit<6>(0x22), Lit<L>(1)).     // jmprt
      IF(inst.get_opcode() == Lit<6>(0x3b), active & pmask).
      IF(inst.get_opcode() == Lit<6>(0x3c), _(top, "mask")).
      ELSE(active).END().ELSE(active);

  bvec<N> out_pc;

  Cassign(out_pc).
    IF(taken).
      IF(inst.get_opcode() == Lit<6>(0x1c)
         || inst.get_opcode() == Lit<6>(0x1e) // jalr[s], jmpr[t]
         || inst.get_opcode() == Lit<6>(0x21)
         || inst.get_opcode() == Lit<6>(0x22), destVal).
      IF(inst.get_opcode() == Lit<6>(0x1b)
         || inst.get_opcode() == Lit<6>(0x1d) // jali[s], jmpi
         || inst.get_opcode() == Lit<6>(0x20), pc+inst.get_imm()).
      IF(inst.get_opcode()==Lit<6>(0x3c) && !_(top,"fallthrough"), _(top,"pc")).
    ELSE(pc).END().ELSE(pc);

  _(_(_(out, "contents"), "warp"), "pc") = Wreg(ldregs, out_pc);

  // Handle the wspawn instruction
  _(_(out, "contents"), "spawn") =
    Wreg(ldregs, inst.get_opcode() == Lit<6>(0x3a));
  _(_(out, "contents"), "spawn_pc") = Wreg(ldregs, rval0);

  tap("branch_full", full);
  tap("branch_out", out);
  tap("branch_in", in);
  HIERARCHY_EXIT();
}

// The stack used in IPDOM pushes two entries at a time and pops one at a time.
// We implement it here with registers.
// It does not have to handle simultaneous pushes and pops.
template <unsigned N, typename T> T IpdomStack(
  node &full, node &empty, node push, node pop, const T& tval, const T& bval
)
{
  bvec<N+1> next_count, count(Reg(next_count));
  Cassign(next_count).
    IF(push, count + Lit<N+1>(2)).
    IF(pop, count - Lit<N+1>(1)).
    ELSE(count);  

  full = (count == Cat(Lit(1), Lit<N>(0)));
  empty = (count == Lit<N+1>(0));

  vec<(1<<N), T> store;
  T top(Mux(Zext<N>(count - Lit<N+1>(1)), store));
  for (unsigned i = 0; i < (1<<N); ++i) {
    T next;
    Cassign(next).
      IF(push && count == Lit<N+1>(i - 1), tval).
      IF(push && count == Lit<N+1>(i), bval).
      ELSE(store[i]);
    store[i] = Reg(next);
  }

  tap("ipdom_stack_count", count);
  tap("ipdom_stack_store", store);
  tap("ipdom_stack_top", top);

  return top;
}
void Execute(splitter_sched_t&, splitter_pred_t&, splitter_reg_t&, reg_func_t&);

enum funcunit_type_t {
  FU_ALU, FU_PLU, FU_MULT, FU_DIV, FU_LSU, FU_BRANCH, FU_PAD0, FU_PAD1, N_FU
};

// Functional Unit Routing Function
void RouteFunc(bvec<N_FU> &valid, const reg_func_int_t &in, node in_valid);

// Functional Units
void Funcunit_alu(func_splitter_t &out, reg_func_t &in);
void Funcunit_plu(func_splitter_t &out, reg_func_t &in);
void Funcunit_mult(func_splitter_t &out, reg_func_t &in);
void Funcunit_div(func_splitter_t &out, reg_func_t &in);
void Funcunit_lsu(func_splitter_t &out, reg_func_t &in);
void Funcunit_branch(func_splitter_t &out, reg_func_t &in);

// The execute pipeline stage
void Execute(splitter_sched_t &out, splitter_pred_t &pwb, splitter_reg_t &rwb,
             reg_func_t &in)
{
  HIERARCHY_ENTER();

  // The input to the functional units includes pre-incremented program counter.
  reg_func_t fu_router_in, fu_router_in_postbuf;
  _(in, "ready") = _(fu_router_in, "ready");
  _(fu_router_in, "valid") = _(in, "valid");
  _(_(_(fu_router_in, "contents"), "warp"), "state")
    = _(_(_(in, "contents"), "warp"), "state");
  _(_(_(fu_router_in, "contents"), "warp"), "active")
    = _(_(_(in, "contents"), "warp"), "active");
  _(_(_(fu_router_in, "contents"), "warp"), "id")
    = _(_(_(in, "contents"), "warp"), "id");
  _(_(_(fu_router_in, "contents"), "warp"), "pc")
    = _(_(_(in, "contents"), "warp"), "pc") + Lit<N>(N/8);
  _(_(fu_router_in, "contents"), "ir") = _(_(in, "contents"), "ir");
  _(_(fu_router_in, "contents"), "pval") = _(_(in, "contents"), "pval");
  _(_(fu_router_in, "contents"), "rval") = _(_(in, "contents"), "rval");

  vec<N_FU, reg_func_t> fu_inputs;
  vec<N_FU, func_splitter_t> fu_outputs;

  Router(fu_inputs, RouteFunc, fu_router_in);

  Funcunit_alu(fu_outputs[FU_ALU], fu_inputs[FU_ALU]);
  Funcunit_plu(fu_outputs[FU_PLU], fu_inputs[FU_PLU]);
  Funcunit_mult(fu_outputs[FU_MULT], fu_inputs[FU_MULT]);
  Funcunit_div(fu_outputs[FU_DIV], fu_inputs[FU_DIV]);
  Funcunit_lsu(fu_outputs[FU_LSU], fu_inputs[FU_LSU]);
  Funcunit_branch(fu_outputs[FU_BRANCH], fu_inputs[FU_BRANCH]);

  func_splitter_t fu_arbiter_out;
  Arbiter(fu_arbiter_out, ArbRR<N_FU>, fu_outputs);

  TAP(fu_inputs);
  TAP(fu_outputs);

  TAP(fu_router_in);
  TAP(fu_router_in_postbuf);
  TAP(fu_arbiter_out);

  // Now we assume the register/predicate writebacks are always ready
  // TODO: Handle ready signal on register/predicate writeback signals
  _(fu_arbiter_out, "ready") = _(out, "ready");

  _(out, "valid") = _(fu_arbiter_out, "valid");
  _(pwb, "valid") = _(fu_arbiter_out, "valid");
  _(rwb, "valid") = _(fu_arbiter_out, "valid");
  _(pwb, "contents") = _(_(fu_arbiter_out, "contents"), "pwb");
  _(rwb, "contents") = _(_(fu_arbiter_out, "contents"), "rwb");
  _(_(out, "contents"), "warp") = _(_(fu_arbiter_out, "contents"), "warp");
  _(_(out, "contents"), "spawn") = _(_(fu_arbiter_out, "contents"), "spawn");
  _(_(out, "contents"), "spawn_pc") =
     _(_(fu_arbiter_out, "contents"), "spawn_pc");

  HIERARCHY_EXIT();
}

void RouteFunc(bvec<N_FU> &valid, const reg_func_int_t &in, node in_valid) {
  HIERARCHY_ENTER();
  harpinst<N, RR, RR> inst(_(in, "ir"));

  bvec<N_FU> v;

  v[FU_ALU] =
    ((inst.get_opcode() == Lit<6>(0x00)   || // nop
      inst.get_opcode() == Lit<6>(0x05)   || // neg
      inst.get_opcode() == Lit<6>(0x06)   || // not
      inst.get_opcode() == Lit<6>(0x07)   || // and
      inst.get_opcode() == Lit<6>(0x08))  || // or
     (inst.get_opcode() == Lit<6>(0x09)   || // xor
      inst.get_opcode() == Lit<6>(0x0a)   || // add
      inst.get_opcode() == Lit<6>(0x0b)   || // sub
      inst.get_opcode() == Lit<6>(0x0f)   || // shl
      inst.get_opcode() == Lit<6>(0x10))) || // shr
    ((inst.get_opcode() == Lit<6>(0x11)   || // andi
      inst.get_opcode() == Lit<6>(0x12)   || // ori
      inst.get_opcode() == Lit<6>(0x13)   || // xori
      inst.get_opcode() == Lit<6>(0x14)   || // addi
      inst.get_opcode() == Lit<6>(0x15))  || // subi
     (inst.get_opcode() == Lit<6>(0x19)   || // shli
      inst.get_opcode() == Lit<6>(0x1a)   || // shri
      inst.get_opcode() == Lit<6>(0x25)   || // ldi
      inst.get_opcode() == Lit<6>(0x1f)));   // clone

  v[FU_PLU] =
    (inst.get_opcode() == Lit<6>(0x26)  || // rtop
     inst.get_opcode() == Lit<6>(0x2b)  || // isneg
     inst.get_opcode() == Lit<6>(0x2c)) || // iszero
   ((inst.get_opcode() == Lit<6>(0x27)  || // andp
     inst.get_opcode() == Lit<6>(0x28)) || // orp
    (inst.get_opcode() == Lit<6>(0x29)  || // xorp
     inst.get_opcode() == Lit<6>(0x2a)));  // notp

  v[FU_MULT] =
    inst.get_opcode() == Lit<6>(0x0c) || // mul
    inst.get_opcode() == Lit<6>(0x16);   // muli

  v[FU_DIV] =
    (inst.get_opcode() == Lit<6>(0x0d)  || // div
     inst.get_opcode() == Lit<6>(0x0e)) || // mod
    (inst.get_opcode() == Lit<6>(0x17)  || // divi
     inst.get_opcode() == Lit<6>(0x18));   // modi

  v[FU_LSU] =
    inst.get_opcode() == Lit<6>(0x23) || // ld
    inst.get_opcode() == Lit<6>(0x24);   // st

  v[FU_BRANCH] =
    (inst.get_opcode() == Lit<6>(0x1b)  || // jali
     inst.get_opcode() == Lit<6>(0x20)  || // jalis
     inst.get_opcode() == Lit<6>(0x1c)  || // jalr
     inst.get_opcode() == Lit<6>(0x21)) || // jalrs
    (inst.get_opcode() == Lit<6>(0x1e)  || // jmpr
     inst.get_opcode() == Lit<6>(0x22)  || // jmprt
     inst.get_opcode() == Lit<6>(0x1d)  || // jmpi
     inst.get_opcode() == Lit<6>(0x3a)) || // wspawn
     inst.get_opcode() == Lit<6>(0x3b)  || // split
     inst.get_opcode() == Lit<6>(0x3c);    // join

  valid = v & bvec<N_FU>(in_valid);
  HIERARCHY_EXIT();
}

void Fetch(fetch_pred_t &out, sched_fetch_t &in, string romFile);

// We're foregoing an icache for now and just using a simple instruction ROM
void Fetch(fetch_pred_t &out, sched_fetch_t &in, string romFile) {
  HIERARCHY_ENTER();
  node ready(_(out, "ready"));

  _(in, "ready") = ready;
  _(out, "valid") = Wreg(ready, _(in, "valid"));
  _(_(out, "contents"), "warp") = Wreg(ready, _(in, "contents"));

  bvec<CLOG2(ROMSZ)> a(
    _(_(in, "contents"), "pc")[range<NN-3, (NN-3)+CLOG2(ROMSZ)-1>()]
  );

  _(_(out, "contents"), "ir") = Wreg(ready,
    LLRom<CLOG2(ROMSZ), N>(a, romFile)
  );
  HIERARCHY_EXIT();
}

// The full Processor
void Harmonica2();

// The pipeline stages
void Sched(sched_fetch_t &out, splitter_sched_t &in);
void Fetch(fetch_pred_t &out, sched_fetch_t &in, string romFile);
void PredRegs(pred_reg_t &out, fetch_pred_t &in, splitter_pred_t &wb);
void GpRegs(reg_func_t &out, pred_reg_t &in, splitter_reg_t &wb);
void Execute(splitter_sched_t&, splitter_pred_t&, splitter_reg_t&, reg_func_t&);

// Implementations
void Harmonica2(string romFile) {
  HIERARCHY_ENTER();

  // Assemble the pipeline
  sched_fetch_t sf;
  fetch_pred_t fp;
  pred_reg_t pr;
  reg_func_t rx;
  splitter_sched_t xs;
  splitter_pred_t xp;
  splitter_reg_t xr;

  Sched(sf, xs);
  Fetch(fp, sf, romFile);
  PredRegs(pr, fp, xp);
  GpRegs(rx, pr, xr);
  Execute(xs, xp, xr, rx);

  TAP(sf); TAP(fp); TAP(pr); TAP(rx); TAP(xs); TAP(xp); TAP(xr);

  Counter("cycles", Lit(1));
  Counter("insts", _(xs, "ready") && _(xs, "valid"));

  HIERARCHY_EXIT();
}

void build() { Harmonica2("harmonica2.hex"); }

void Funcunit_lsu(func_splitter_t &out, reg_func_t &in);

void MemSystem(mem_resp_t &out, mem_req_t &in);

// Load/store unit
void Funcunit_lsu(func_splitter_t &out, reg_func_t &in)
{
  HIERARCHY_ENTER();

  mem_req_t req;
  mem_resp_t resp;

  harpinst<N, RR, RR> inst(_(_(in, "contents"), "ir"));

  bvec<L> active(_(_(_(in, "contents"), "warp"), "active"));

  // Connect "in" to "req"
  _(in, "ready") = _(req, "ready");
  _(req, "valid") = _(in, "valid");
  _(_(req, "contents"), "warp") = _(_(in, "contents"), "warp");
  _(_(req, "contents"), "wr") = inst.is_store();
  _(_(req, "contents"), "mask") =
    _(_(_(in, "contents"), "pval"), "pmask") & active;
  for (unsigned l = 0; l < L; ++l) {
    _(_(req, "contents"), "a")[l] =
      Mux(inst.is_store(), 
        _(_(_(in, "contents"), "rval"), "val0")[l] + inst.get_imm(),
        _(_(_(in, "contents"), "rval"), "val1")[l] + inst.get_imm()
      );
    _(_(req, "contents"), "d")[l] =
      _(_(_(in, "contents"), "rval"), "val0")[l];
  }

  // Keep track of whether operations issued to memory system were stores
  node issue(_(in, "ready") && _(in, "valid"));
  bvec<L> ldMask(Wreg(issue,
    bvec<L>(inst.has_rdst()) & _(_(_(in, "contents"), "pval"), "pmask")
  ));
  bvec<RR> ldDest(Wreg(issue, inst.get_rdst()));

  // Connect "out" to resp
  _(resp, "ready") = _(out, "ready");
  _(out, "valid") = _(resp, "valid");
  _(_(out, "contents"), "warp") = _(_(resp, "contents"), "warp");
  _(_(_(out,"contents"),"rwb"),"wid") = _(_(_(resp,"contents"),"warp"),"id");
  _(_(_(out,"contents"),"rwb"),"mask") = ldMask;
  _(_(_(out,"contents"),"rwb"),"dest") = ldDest;
  _(_(_(out,"contents"),"rwb"),"val") = _(_(resp,"contents"),"q");

  MemSystem(resp, req);

  tap("lsu_out", out);
  tap("lsu_in", in);

  HIERARCHY_EXIT();
}

void DummyCache(cache_resp_t &out, cache_req_t &in) {
  const unsigned DCS(CLOG2(DUMMYCACHE_SZ));

  node ready = _(out, "ready");
  _(in, "ready") = ready;

  node ldregs(ready && _(in, "valid")), next_full, full(Reg(next_full));

  Cassign(next_full).
    IF(!full && _(in, "valid") && ready, Lit(1)).
    IF(full && (!_(in, "valid") || !ready), Lit(0)).
    ELSE(full);

  _(out, "valid") = full;
  _(_(out, "contents"), "warp") = Wreg(ldregs, _(_(in, "contents"), "warp"));

  bvec<N> a(_(_(in, "contents"), "a"));
  bvec<CLOG2(DCS)> devAddr(
    a[range<CLOG2(LINE*(N/8)), CLOG2(LINE*(N/8))+CLOG2(DCS)-1>()]
  );

  vec<LINE, bvec<N> > memd(_(_(in, "contents"), "d"));
  vec<LINE, bvec<N> > memq;
  for (unsigned i = 0; i < LINE; ++i) {
    node wr(ready && _(in, "valid") && _(_(in, "contents"), "mask")[i] &&
              _(_(in, "contents"), "wr"));

    memq[i] = Syncmem(devAddr, memd[i], wr);

    if (i == 0 && SOFT_IO && !FPGA) {
      static unsigned consoleOutVal;
      node wrConsole(wr && a[N-1]);
      EgressInt(consoleOutVal, memd[i]);
      EgressFunc(
        [](bool x){if (x) cout << char(consoleOutVal);},
        wrConsole
      );

      TAP(wrConsole);
      tap("consoleVal", memd[i]);
    }

    if (i == 0 && FPGA && FPGA_IO) {    
      node wrConsole(Reg(wr && a[N-1]));
      OUTPUT(wrConsole);
      bvec<8> consoleOutVal(Wreg(wr && a[N-1], memd[i][range<0,7>()]));
      OUTPUT(consoleOutVal);
    }
  }

  if (!FPGA && DEBUG_MEM) {
    static unsigned long addrVal, dVal[LINE], qVal[LINE], warpId;
    static bool wrVal, maskVal[LINE];
    Egress(wrVal, Reg(ready && _(in, "valid") && _(_(in, "contents"), "wr")));
    for (unsigned l = 0; l < LINE; ++l) {
      EgressInt(dVal[l], Reg(memd[l]));
      Egress(maskVal[l], Reg(_(_(in, "contents"), "mask")[l]));
      EgressInt(qVal[l], memq[l]);
    }

    EgressInt(addrVal, Reg(a));
    EgressInt(warpId, Reg(_(_(_(in, "contents"), "warp"), "id")));
    EgressFunc([](bool x) {
      if (x) {
        cout << warpId << ": Mem " << (wrVal?"store":"load") << ':' << endl;
        for (unsigned l = 0; l < LINE; ++l) {
          if (maskVal[l]) {
            cout << "  0x" << hex << addrVal + l*(N/8);
            if (wrVal) cout << hex << ": 0x" << dVal[l];
            else cout << hex << ": 0x" << qVal[l];
            cout << endl;
          }
        }
      }
    }, Reg(ready && _(in, "valid")));
  }

  vec<LINE, bvec<N> > held_memq;
  Flatten(held_memq) = Wreg(ldregs, Flatten(memq));

  tap("dummy_cache_memq", memq);
  TAP(devAddr);

  Flatten(_(_(out, "contents"), "q")) =
    Mux(ready && Reg(ready), Flatten(held_memq), Flatten(memq));

  _(_(out, "contents"), "lane") = Wreg(ldregs, _(_(in, "contents"), "lane"));  
}

void MemSystem(mem_resp_t &out, mem_req_t &in) {
  HIERARCHY_ENTER();
  // TODO: Add an associative table to enable cache_resps to arrive out-of-order
  // e.g. from a non-blocking cache.

  node next_full, full(Reg(next_full)), fill, empty;
  _(in, "ready") = !full;
  next_full = (full && !empty) || (!full && fill && !empty);
  fill = _(in, "valid") && !full;

  vec<L, bvec<L> > eqmat; // Coalesce matrix: which addresses are equal?
  bvec<L> covered, // Is my request covered by that of a prior lane?
          mask(_(_(in, "contents"), "mask"));
  cache_req_t cache_req;
  cache_resp_t cache_resp;

  vec<L, bvec<N> > a(_(_(in, "contents"), "a"));

  for (unsigned i = 0; i < L; ++i) {
    for (unsigned j = i; j < L; ++j)
      eqmat[i][j] = Lit(0);
    for (unsigned j = 0; j < i; ++j) {
      bvec<N-CLOG2(LINE*(N/8))> ai(a[i][range<CLOG2(LINE*(N/8)), N-1>()]),
                                aj(a[j][range<CLOG2(LINE*(N/8)), N-1>()]);
      eqmat[i][j] = ai == aj && mask[i] && mask[j];
    }
    covered[i] = OrN(eqmat[i]);
  }

  bvec<L> allReqMask(Wreg(fill, ~covered & mask)),
          next_sentReqMask, sentReqMask(Reg(next_sentReqMask)),
          next_returnedReqMask, returnedReqMask(Reg(next_returnedReqMask));

  bvec<L> ldqReg;
  vec<L, bvec<L> > eqmatReg;
  vec<L, bvec<N> > aReg, dReg, qReg;
  for (unsigned l = 0; l < L; ++l) {
    for (unsigned i = 0; i < L; ++i) {
      if (i == l) eqmatReg[l][i] = Lit(1);
      else        eqmatReg[l][i] = Wreg(fill, eqmat[i][l]);
    }
    aReg[l] = Wreg(fill, a[l]);
    dReg[l] = Wreg(fill, _(_(in, "contents"), "d")[l]);
    qReg[l] = Wreg(ldqReg[l] && _(cache_resp,"valid") && _(cache_resp,"ready"),
                   Mux(aReg[l][range<NN-3, NN-3+CLOG2(LINE)-1>()],
                       _(_(cache_resp,"contents"),"q")) >>
                   Cat(aReg[l][range<0, NN-3 - 1>()], Lit<3>(0)));
  }

  bvec<LL> sel(Lsb(allReqMask & ~sentReqMask));

  TAP(eqmat); TAP(covered); TAP(allReqMask); TAP(sentReqMask);
  TAP(returnedReqMask); TAP(sel);

  Cassign(next_sentReqMask).
    IF(fill, Lit<L>(0)).
    IF(_(cache_req, "ready") && (sentReqMask != allReqMask),
       sentReqMask | Lit<L>(1)<<sel).
    ELSE(sentReqMask);

  bvec<N> reqAddr(Mux(sel, aReg) & ~Lit<N>(LINE*(N/8)-1));

  _(_(cache_req, "contents"), "a") = reqAddr;
  _(_(cache_req, "contents"), "lane") = sel;
  _(_(cache_req, "contents"), "warp") = Wreg(fill, _(_(in, "contents"),"warp"));
  _(_(cache_req, "contents"), "wr") = Wreg(fill, _(_(in, "contents"), "wr"));

  tap("mem_aReg", aReg);  tap("mem_dReg", dReg); tap("mem_qReg", qReg);

  for (unsigned i = 0; i < LINE; ++i) {
    const unsigned NB(CLOG2(N/8)), // log2(N), expressed in (8-bit) bytes
                   LB(NB + CLOG2(LINE)); // log2(bytes in a cache line)

    bvec<L> maskBits;
    for (unsigned l = 0; l < L; ++l)
      maskBits[l] =
        (aReg[l][range<NB, LB-1>()] == Lit<CLOG2(LINE)>(i)) &&
        (aReg[l][range<LB, N-1>()] == reqAddr[range<LB, N-1>()]) &&
        mask[l] && _(_(_(in, "contents"), "warp"), "active")[l];
    
    _(_(cache_req, "contents"), "mask")[i] = OrN(maskBits);
    _(_(cache_req, "contents"), "d")[i] = Mux(Log2(maskBits), dReg);

    ostringstream oss;
    oss << "maskBits" << i;
    tap(oss.str(), maskBits);
  }

  TAP(cache_req);
  TAP(cache_resp);

  _(cache_req, "valid") = full && sentReqMask != allReqMask;

  DummyCache(cache_resp, cache_req);

  _(cache_resp, "ready") = full && returnedReqMask != allReqMask;

  Cassign(next_returnedReqMask).
    IF(fill, Lit<L>(0)).
    IF(_(cache_resp, "ready") && _(cache_resp, "valid"),
      returnedReqMask | Lit<L>(1)<<_(_(cache_resp, "contents"),"lane")).
    ELSE(returnedReqMask);

  // Load these lanes' q registers for this response.
  ldqReg = Mux(_(_(cache_resp,"contents"),"lane"), eqmatReg);

  ASSERT(!OrN(returnedReqMask & ~sentReqMask));

  _(out, "valid") = full && (returnedReqMask == allReqMask);

  _(_(out, "contents"), "warp") = Wreg(fill, _(_(in, "contents"), "warp"));
  _(_(out, "contents"), "q") = qReg;

  empty = _(out, "valid") && _(out, "ready");

  tap("mem_fill", fill);
  tap("mem_empty", empty);
  tap("mem_full", full);
  tap("mem_in", in);
  tap("mem_out", out);
  tap("mem_reqAddr", reqAddr);
  tap("mem_ldqReg", ldqReg);
  tap("mem_eqmatReg", eqmatReg);

  HIERARCHY_EXIT();
}

template <unsigned N>
   bvec<N> SerialMul(node &busy, bvec<N> a, bvec<N> b, node &start);

template <unsigned N>
  void SerialDiv(
    bvec<N> &q, bvec<N> &r, node &ready, chdl::node &waiting,
    bvec<N> n, bvec<N> d, node start, node stall
  );

void Funcunit_mult(func_splitter_t &out, reg_func_t &in);
void Funcunit_div(func_splitter_t &out, reg_func_t &in);

void Funcunit_mult(func_splitter_t &out, reg_func_t &in) {
  HIERARCHY_ENTER();
  harpinst<N, RR, RR> inst(_(_(in, "contents"), "ir"));
  bvec<L> active(_(_(_(in, "contents"), "warp"), "active")), busy;

  node imm(inst.get_opcode()[4]), start,
       next_full, full(Reg(next_full));

  bvec<N> immVal(inst.get_imm());

  vec<L, bvec<N> > a(_(_(_(in,"contents"),"rval"),"val0")),
                   b(_(_(_(in,"contents"),"rval"),"val1")),
                   mulOut;

  for (unsigned l = 0; l < L; ++l)
    mulOut[l] = SerialMul(busy[l], a[l], Mux(imm, b[l], immVal), start);

  Cassign(next_full).
    IF(start, Lit(1)).
    IF(!OrN(busy) && _(out, "ready"), Lit(0)).
    ELSE(full);

  _(in, "ready") = !OrN(busy) && !full;
  start = _(in, "valid") && _(in, "ready");
  _(out, "valid") = full && !OrN(busy);

  bvec<L> pmask(_(_(_(in, "contents"), "pval"), "pmask"));

  _(_(out, "contents"), "warp") = Wreg(start, _(_(in, "contents"), "warp"));
  _(_(_(out, "contents"), "rwb"), "dest") = Wreg(start, inst.get_rdst());
  _(_(_(out, "contents"), "rwb"), "mask") = Wreg(start, pmask & active);
  _(_(_(out, "contents"), "rwb"), "val") = mulOut;
  _(_(_(out, "contents"), "rwb"), "wid") =
    Wreg(start, _(_(_(in, "contents"), "warp"), "id"));

  tap("mul_full", full);
  tap("mul_busy", OrN(busy));
  tap("mul_start", start);
  tap("mul_in", in);
  tap("mul_out", out);

  HIERARCHY_EXIT();
}

void Funcunit_div(func_splitter_t &out, reg_func_t &in) {
  HIERARCHY_ENTER();
  harpinst<N, RR, RR> inst(_(_(in, "contents"), "ir"));

  node imm(inst.get_opcode()[4]), start;

  bvec<L> ready, valid;

  bvec<N> immVal(inst.get_imm());

  vec<L, bvec<N> > a(_(_(_(in,"contents"),"rval"),"val0")),
                   b(_(_(_(in,"contents"),"rval"),"val1")),
                   q, r;

  bvec<L> active(_(_(_(in, "contents"), "warp"), "active"));

  for (unsigned l = 0; l < L; ++l) {
    SerialDiv(q[l], r[l], valid[l], ready[l],
              a[l], Mux(imm, b[l], immVal), start, !_(out, "ready"));
  }

  _(in, "ready") = AndN(ready);
  start = _(in, "valid") && _(in, "ready");
  _(out, "valid") = AndN(valid);

  bvec<L> pmask(_(_(_(in, "contents"), "pval"), "pmask"));

  _(_(out, "contents"), "warp") = Wreg(start, _(_(in, "contents"), "warp"));
  _(_(_(out, "contents"), "rwb"), "dest") = Wreg(start, inst.get_rdst());
  _(_(_(out, "contents"), "rwb"), "mask") = Wreg(start, pmask & active);
  _(_(_(out, "contents"), "rwb"), "wid") =
    Wreg(start, _(_(_(in, "contents"), "warp"), "id"));
  for (unsigned l = 0; l < L; ++l)
    _(_(_(out, "contents"), "rwb"), "val")[l] =
      Mux(Wreg(start, inst.get_opcode()[0]), r[l], q[l]);

  tap("div_valid", valid);
  tap("div_ready", ready);
  tap("div_start", start);
  tap("div_in", in);
  tap("div_out", out);

  HIERARCHY_EXIT();
}

template <unsigned N>
  bvec<N> SerialMul(node &busy, bvec<N> a, bvec<N> b, node &start)
{
  bvec<N> shrreg;
  bvec<CLOG2(N+1)> next_state, state(Reg(next_state));
  Cassign(next_state).
    IF(state == Lit<CLOG2(N+1)>(0)).
    IF(start, Lit<CLOG2(N+1)>(2)).
    ELSE(Lit<CLOG2(N+1)>(0)).
    END().
    IF(state == Lit<CLOG2(N+1)>(N), Lit<CLOG2(N+1)>(0)).
    IF(OrN(shrreg), state + Lit<CLOG2(N+1)>(2)).
    ELSE(Lit<CLOG2(N+1)>(0));

  busy = OrN(state);

  bvec<N> next_shlreg, shlreg(Reg(next_shlreg));
  Cassign(next_shlreg).
    IF(start, b).
    ELSE(shlreg << Lit<CLOG2(N)>(2));

  bvec<N> next_shrreg;
  shrreg = Reg(next_shrreg);
  Cassign(next_shrreg).
    IF(start, a).
    ELSE(shrreg >> Lit<CLOG2(N)>(2));

  bvec<N> shlreg2(Cat(shlreg[range<0,N-2>()], Lit(0))),
          shlreg3(shlreg2 + shlreg);

  bvec<N> next_resultreg, resultreg(Reg(next_resultreg));
  Cassign(next_resultreg).
    IF(start, Lit<N>(0)).
    IF(busy && shrreg[0] && !shrreg[1], resultreg + shlreg).
    IF(busy && !shrreg[0] && shrreg[1], resultreg + shlreg2).
    IF(busy && shrreg[0] && shrreg[1], resultreg + shlreg3).
    ELSE(resultreg);

  static bool tapped(false);
  if (!tapped) {
    tap("mul_resultreg", resultreg);
    tap("mul_state", state);
    tap("mul_shrreg", shrreg);
    tap("mul_shlreg", shlreg);
    tap("mul_shlreg2", shlreg2);
    tap("mul_shlreg3", shlreg3);
    tapped = true;
  }

  return resultreg;
}

template <unsigned N, bool D>
  chdl::bvec<N> Shiftreg(
    chdl::bvec<N> in, chdl::node load, chdl::node shift, chdl::node shin
  )
{
  using namespace chdl;
  using namespace std;

  HIERARCHY_ENTER();  

  bvec<N+1> val;
  val[D?N:0] = shin;

  if (D) {
    for (int i = N-1; i >= 0; --i)
      val[i] = Reg(Mux(load, Mux(shift, val[i], val[i+1]), in[i]));
    HIERARCHY_EXIT();
    return val[range<0, N-1>()];
  } else {
    for (unsigned i = 1; i < N; ++i)
      val[i] = Reg(Mux(load, Mux(shift, val[i], val[i-1]), in[i-1]));
    HIERARCHY_EXIT();
    return val[range<1, N>()];
  }
}

template <unsigned N>
  chdl::bvec<N> Lshiftreg(
    chdl::bvec<N> in, chdl::node load, chdl::node shift,
    chdl::node shin = chdl::Lit(0)
  )
{ return Shiftreg<N, false>(in, load, shift, shin); }

template <unsigned N>
  chdl::bvec<N> Rshiftreg(
    chdl::bvec<N> in, chdl::node load, chdl::node shift,
    chdl::node shin = chdl::Lit(0)
  )
{ return Shiftreg<N, true>(in, load, shift, shin); }

template <unsigned N>
  void SerialDiv(
    bvec<N> &q, bvec<N> &r, node &ready, node &waiting,
    bvec<N> n, bvec<N> d, node start, node stall
  )
{
  // The controller
  bvec<CLOG2(N+3)> next_state, state(Reg(next_state));
  Cassign(next_state).
    IF(state == Lit<CLOG2(N+3)>(0)).
      IF(start, Lit<CLOG2(N+3)>(1)).
      ELSE(Lit<CLOG2(N+3)>(0)).
    END().IF(state == Lit<CLOG2(N+3)>(N+2)).
      IF(stall, Lit<CLOG2(N+3)>(N+2)).
      ELSE(Lit<CLOG2(N+3)>(0)).
    END().ELSE(state + Lit<CLOG2(N+3)>(1));

  tap("div_state", state);

  ready = (state == Lit<CLOG2(N+3)>(N+2));
  waiting = (state == Lit<CLOG2(N+3)>(0));

  tap("div_waiting", waiting);

  // The data path
  bvec<2*N> s(Rshiftreg(Cat(d, Lit<N>(0)), start, Lit(1)));
  node qbit(Cat(Lit<N>(0), r) >= s);
  r = Reg(Mux(start, Mux(qbit, r, r - s[range<0, N-1>()]), n));
  q = Lshiftreg(Lit<N>(0), start, !ready, qbit);

  tap("div_s", s);
  tap("div_r", r);
  tap("div_q", q);
}

void PredRegs(pred_reg_t &out, fetch_pred_t &in, splitter_pred_t &wb);
void GpRegs(reg_func_t &out, pred_reg_t &in, splitter_reg_t &wb);

void PredRegs(pred_reg_t &out, fetch_pred_t &in, splitter_pred_t &wb) {
  HIERARCHY_ENTER();
  harpinst<N, RR, RR> inst(_(_(in, "contents"), "ir"));

  bvec<WW> wid(_(_(_(in, "contents"), "warp"), "id")),
           wb_wid(_(_(wb, "contents"), "wid"));

  bvec<WW + RR> a_ipred(Cat(wid, inst.get_pred())),
                a_src0 (Cat(wid, inst.get_psrc0())),
                a_src1 (Cat(wid, inst.get_psrc1())),
                a_wb   (Cat(wb_wid, _(_(wb, "contents"), "dest")));

  bvec<L> wb_mask(_(_(wb, "contents"), "mask") & bvec<L>(_(wb, "valid"))),
          wb_val(_(_(wb, "contents"), "val"));

  vec<3, bvec<WW + RR> > rd_addr;
  rd_addr[0] = a_ipred;
  rd_addr[1] = a_src0;
  rd_addr[2] = a_src1;

  vec<L, vec<3, bvec<1> > > q;

  for (unsigned i = 0; i < L; ++i)
    q[i] = Syncmem(rd_addr, bvec<1>(wb_val[i]), a_wb, wb_mask[i]);
 
  bvec<L> pval0, pval1, pmask;

  for (unsigned i = 0; i < L; ++i) {
    pmask[i] = q[i][0][0];
    pval0[i] = q[i][1][0];
    pval1[i] = q[i][2][0];
  }

  _(wb, "ready") = Lit(1);  // Always ready to accept writebacks.

  tap("pred_wb_wid", _(_(wb, "contents"), "wid"));

  // Handle ready and valid signals
  node ready(_(out, "ready"));
  _(in, "ready") = ready;
  _(out, "valid") = Wreg(ready, _(in, "valid"));
  _(_(out, "contents"), "warp") = Wreg(ready, _(_(in, "contents"), "warp"));
  _(_(out, "contents"), "ir") = Wreg(ready, _(_(in, "contents"), "ir"));

  // The mask should be all 1s if the instruction is not predicated.
  _(_(_(out, "contents"), "pval"), "pmask") 
    = Latch(!ready, pmask | bvec<L>(Reg(!inst.has_pred())));

  _(_(_(out, "contents"), "pval"), "val0") = Latch(!ready, pval0);
  _(_(_(out, "contents"), "pval"), "val1") = Latch(!ready, pval1);

  HIERARCHY_EXIT();
}

void GpRegs(reg_func_t &out_buffered, pred_reg_t &in, splitter_reg_t &wb) {
  HIERARCHY_ENTER();
  reg_func_t out;

  harpinst<N, RR, RR> inst(_(_(in, "contents"), "ir"));

  bvec<RR> wb_dest(_(_(wb, "contents"), "dest"));
  bvec<WW> wid(_(_(_(in, "contents"), "warp"), "id")),
           wb_wid(_(_(wb, "contents"), "wid"));
  bvec<LL> wb_clonesrc(_(_(wb, "contents"), "clonesrc")),
           wb_clonedest(_(_(wb, "contents"), "clonedest"));
  vec<L, bvec<N> > wb_val(_(_(wb, "contents"), "val"));
  bvec<L> wbMask(_(_(wb, "contents"), "mask"));

  vec<R, bvec<N> > clonebus;

  _(wb, "ready") = Lit(1); // Always ready to accept writebacks.

  vec<L, bvec<N> > rval0, rval1, rval2;

  node clone(_(wb, "valid") && _(_(wb, "contents"), "clone"));
  bvec<L> wb_mask(_(_(wb, "contents"), "mask") &
                    bvec<L>(_(wb, "valid") && !clone));

  vec<L, vec<R, vec<2, bvec<N> > > > q;

  vec<2, bvec<WW> > a;
  a[0] = Mux(clone, wid, wb_wid);
  a[1] = wb_wid;

  for (unsigned l = 0; l < L; ++l) {
    for (unsigned i = 0; i < R; ++i) {
      node wr(wb_mask[l] && wb_dest == Lit<RR>(i) ||
                wb_clonedest == Lit<LL>(l) && clone);
      if (SRAM_REGS) {
        q[l][i] = Syncmem(a, Mux(clone, wb_val[l], clonebus[i]), wb_wid, wr);
      } else {
        vec<W, bvec<N> > regs;
        for (unsigned w = 0; w < W; ++w) {
          unsigned initialVal(0);
          if (i == 0) initialVal = l;
          if (i == 1) initialVal = w;

          regs[w] = Wreg(
            wr && wb_wid == Lit<WW>(w),
            Mux(clone, wb_val[l], clonebus[i]),
            initialVal
          );

          ostringstream rname;
          rname << "reg_" << w << '_' << l << '_' << i;
          tap(rname.str(), regs[w]);
        }
        q[l][i][0] = Reg(Mux(a[0], regs));
        q[l][i][1] = Reg(Mux(a[1], regs));
      }
    }
  }

  for (unsigned i = 0; i < R; ++i) {
    vec<L, bvec<N> > cb_slice;
    for (unsigned l = 0; l < L; ++l)
      cb_slice[l] = q[l][i][1];
    clonebus[i] = Mux(wb_clonesrc, cb_slice);
  }


  for (unsigned l = 0; l < L; ++l) {
    rval0[l] = Mux(Reg(inst.get_rsrc0()), q[l])[0];
    rval1[l] = Mux(Reg(inst.get_rsrc1()), q[l])[0];
    rval2[l] = Mux(Reg(inst.get_rsrc2()), q[l])[0];
  }

  TAP(rval0);
  TAP(rval1);
  TAP(rval2);

  TAP(wb_wid);

  node ready(_(out, "ready"));

  _(in, "ready") = ready;
  _(out, "valid") = Wreg(ready, _(in, "valid"));
  _(_(out, "contents"), "warp") = Wreg(ready, _(_(in, "contents"), "warp"));
  _(_(out, "contents"), "ir") = Wreg(ready, _(_(in, "contents"), "ir"));
  _(_(out, "contents"), "pval") = Wreg(ready, _(_(in, "contents"), "pval"));
  Flatten(_(_(_(out,"contents"),"rval"),"val0")) = Latch(!ready,Flatten(rval0));
  Flatten(_(_(_(out,"contents"),"rval"),"val1")) = Latch(!ready,Flatten(rval1));
  Flatten(_(_(_(out,"contents"),"rval"),"val2")) = Latch(!ready,Flatten(rval2));

  Buffer<1>(out_buffered, out);

  HIERARCHY_EXIT();
}

// The full Processor
void Harmonica2();

// The pipeline stages
void Sched(sched_fetch_t &out, splitter_sched_t &in);

// Inject the initial warp
void Starter(splitter_sched_t &out) {
  HIERARCHY_ENTER();
  node firstCyc(Reg(Lit(0), 1));

  _(out, "valid") = firstCyc;
  // TODO: TS_KERNEL
  _(_(_(out, "contents"), "warp"), "state") = Lit<SS>(TS_USER);
  _(_(_(out, "contents"), "warp"), "pc") = Lit<N>(0);
  _(_(_(out, "contents"), "warp"), "active") = Lit<L>(1);
  _(_(_(out, "contents"), "warp"), "id") = Lit<WW>(0);

  ASSERT(!(firstCyc && !_(out, "ready")));
  HIERARCHY_EXIT();
}

void Sched(sched_fetch_t &out, splitter_sched_t &in) {
  HIERARCHY_ENTER();
  splitter_sched_t starter_out, arb_out; Starter(starter_out);
  sched_fetch_t buf_in;
  vec<2, splitter_sched_t> arb_in = {in, starter_out};
  Arbiter(arb_out, ArbUniq<2>, arb_in);
  TAP(starter_out);

  node next_spawnState, spawnState(Reg(next_spawnState)),
       ldspawn(!spawnState && _(buf_in, "ready") && _(in, "valid")
               && _(_(in, "contents"), "spawn"));


  Cassign(next_spawnState).
    IF(spawnState && _(buf_in, "ready"), Lit(0)).
    IF(ldspawn, Lit(1)).
    ELSE(spawnState);

  
  warp_t spawnWarp;
  _(spawnWarp, "state") = Lit<SS>(TS_USER);
  _(spawnWarp, "pc") = Wreg(ldspawn, _(_(in, "contents"), "spawn_pc"));
  _(spawnWarp, "active") = Lit<L>(1);
  _(spawnWarp, "id") = Wreg(ldspawn, _(spawnWarp, "id") + Lit<WW>(1));

  _(arb_out, "ready") = _(buf_in, "ready") && !spawnState;
  _(buf_in, "valid") = _(arb_out, "valid") || spawnState;
  const unsigned WARPSZ(sz<warp_t>::value);
  _(buf_in, "contents") = Mux(spawnState,
                            bvec<WARPSZ>(_(_(arb_out, "contents"), "warp")),
                            bvec<WARPSZ>(spawnWarp));

  Buffer<WW>(out, buf_in);
  TAP(buf_in); TAP(in);
  TAP(spawnState);
  TAP(next_spawnState);
  HIERARCHY_EXIT();
}
