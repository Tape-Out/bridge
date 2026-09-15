package BridgeTb;

// bridge 的库包测试台：测试台里的 APB4 主机经四座桥访问下游。下游两种 AXI4-Lite 完成方、两种 TL-UL 完成方：
// 库里的 BindT 包一个会停顿三拍的寄存器目标；测试台手写的刁难型——AXI 的 AWREADY 要等 WVALID 先来、
// ARREADY 隔拍才高、答复晚几拍，TL-UL 的 d_valid 与答复都从同一拍的 A 通道组合出来（TileLink 4.1 允许）。
// 下游的发起方引脚每拍查握手规矩。判据见 notes/规范对照/bridge.md。

import StmtFSM::*;
import ConfigReg::*;
import Vector::*;
import RegIf::*;
import Apb4::*;
import Axi4Lite::*;
import Tilelink::*;
import Bridge::*;

// 会停顿 n 拍的寄存器目标：0x04 值寄存器（按选通合并）、0x08 写次数、0x0C 总是回错
module mkSlowDev#(Integer n)(RegTarget#(8, 32));
  Reg#(Bit#(8))        cnt[2]  <- mkCReg(2, 0);
  Reg#(Bool)           busy[2] <- mkCReg(2, False);
  Reg#(Bool)           ansV[2] <- mkCReg(2, False);
  Reg#(RegReq#(8, 32)) q[2]    <- mkCReg(2, unpack(0));
  Reg#(RegRsp#(32))    ans[2]  <- mkCReg(2, unpack(0));
  Reg#(Bit#(32))       v       <- mkReg(0);
  Reg#(Bit#(16))       wc      <- mkReg(0);

  rule step;
    let r = q[0];
    if (busy[0] && cnt[0] == 0) begin
      Bool er = r.addr == 8'h0C;
      busy[0] <= False;
      ansV[0] <= True;
      ans[0]  <= RegRsp { rdata: r.addr == 8'h08 ? zeroExtend(wc) : v, err: er };
      if (r.write && !er) begin
        wc <= wc + 1;
        if (r.addr == 8'h04) v <= applyStrb(v, r.wdata, r.wstrb);
      end
    end else begin
      if (busy[0]) cnt[0] <= cnt[0] - 1;
      ansV[0] <= False;
    end
  endrule

  method Action req(Bool valid, RegReq#(8, 32) r);
    if (valid && !busy[1] && !ansV[1]) begin
      busy[1] <= True;
      cnt[1]  <= fromInteger(n);
      q[1]    <= r;
    end
  endmethod
  method Bool ready = !busy[1] && !ansV[1];
  method Bool rspValid = ansV[1];
  method RegRsp#(32) rsp = ans[1];
endmodule

// 0x0C 回 SLVERR、0x10 回 DECERR，其余 OKAY
function Bit#(2) axiRespAt(Bit#(8) a) = a == 8'h0C ? 2'b10 : (a == 8'h10 ? 2'b11 : 2'b00);

// 刁难型 AXI4-Lite 完成方
module mkPickyAxi(Axi4LiteSlavePins#(8, 32));
  Wire#(Tuple2#(Bool, Bit#(8)))           awIn <- mkBypassWire;
  Wire#(Tuple3#(Bool, Bit#(32), Bit#(4))) wIn  <- mkBypassWire;
  Wire#(Bool)                             bIn  <- mkBypassWire;
  Wire#(Tuple2#(Bool, Bit#(8)))           arIn <- mkBypassWire;
  Wire#(Bool)                             rIn  <- mkBypassWire;

  Reg#(Maybe#(Bit#(8)))                    awH   <- mkReg(tagged Invalid);
  Reg#(Maybe#(Tuple2#(Bit#(32), Bit#(4)))) wH    <- mkReg(tagged Invalid);
  Reg#(Bool)                               wSeen <- mkReg(False);
  Reg#(UInt#(3))                           bWait <- mkReg(0);
  Reg#(Bool)                               bV    <- mkReg(False);
  Reg#(Bit#(2))                            bR    <- mkReg(0);
  Reg#(Bool)                               tick  <- mkReg(False);
  Reg#(Maybe#(Bit#(8)))                    arH   <- mkReg(tagged Invalid);
  Reg#(UInt#(3))                           rWait <- mkReg(0);
  Reg#(Bool)                               rV    <- mkReg(False);
  Reg#(Bit#(32))                           rD    <- mkReg(0);
  Reg#(Bit#(2))                            rR    <- mkReg(0);
  Reg#(Bit#(32))                           val   <- mkReg(0);
  Reg#(Bit#(16))                           wc    <- mkReg(0);

  // WVALID 先来过一拍才给 AWREADY：发起方要是等 AWREADY 才抬 WVALID，这里永远不给，卡死
  Bool awRdy = !isValid(awH) && !bV && wSeen;
  Bool wRdy  = !isValid(wH) && !bV;
  // ARREADY 隔拍才高：握手之前 ARVALID 与 ARADDR 要顶住
  Bool arRdy = !isValid(arH) && !rV && tick;

  rule step;
    match {.awv, .awa} = awIn;
    match {.wv, .wd, .ws} = wIn;
    match {.arv, .ara} = arIn;
    tick <= !tick;

    let      nAw   = (awv && awRdy) ? tagged Valid awa : awH;
    let      nW    = (wv && wRdy) ? tagged Valid tuple2(wd, ws) : wH;
    Bool     nSeen = wSeen || wv;
    Bool     nbV   = bV && !bIn;
    Bit#(2)  nbR   = bR;
    UInt#(3) nbW   = bWait;
    Bit#(32) nval  = val;
    Bit#(16) nwc   = wc;
    // 写：两次握手凑齐之后再等三拍出 B
    if (isValid(nAw) && isValid(nW) && !bV) begin
      if (bWait == 3) begin
        Bit#(8) a = fromMaybe(?, nAw);
        match {.d, .s} = fromMaybe(?, nW);
        nbV = True; nbR = axiRespAt(a);
        if (axiRespAt(a) == 2'b00) begin
          nwc = wc + 1;
          if (a == 8'h04) nval = applyStrb(val, d, s);
        end
        nAw = tagged Invalid; nW = tagged Invalid; nbW = 0; nSeen = False;
      end else nbW = bWait + 1;
    end

    // 读：AR 握手之后再等两拍出 R
    let      nAr = (arv && arRdy) ? tagged Valid ara : arH;
    Bool     nrV = rV && !rIn;
    Bit#(32) nrD = rD;
    Bit#(2)  nrR = rR;
    UInt#(3) nrW = rWait;
    if (nAr matches tagged Valid .a &&& !rV) begin
      if (rWait == 2) begin
        nrV = True; nrR = axiRespAt(a); nrD = a == 8'h08 ? zeroExtend(nwc) : nval;
        nAr = tagged Invalid; nrW = 0;
      end else nrW = rWait + 1;
    end

    awH <= nAw; wH <= nW; wSeen <= nSeen; bV <= nbV; bR <= nbR; bWait <= nbW;
    arH <= nAr; rV <= nrV; rD <= nrD; rR <= nrR; rWait <= nrW; val <= nval; wc <= nwc;
  endrule

  method Action aw_in(Bool v, Bit#(8) a, Bit#(3) p); awIn <= tuple2(v, a); endmethod
  method Bool awready = awRdy;
  method Action w_in(Bool v, Bit#(32) d, Bit#(4) s); wIn <= tuple3(v, d, s); endmethod
  method Bool wready = wRdy;
  method Bool bvalid = bV;
  method Bit#(2) bresp = bR;
  method Action b_in(Bool r); bIn <= r; endmethod
  method Action ar_in(Bool v, Bit#(8) a, Bit#(3) p); arIn <= tuple2(v, a); endmethod
  method Bool arready = arRdy;
  method Bool rvalid = rV;
  method Bit#(32) rdata = rD;
  method Bit#(2) rresp = rR;
  method Action r_in(Bool r); rIn <= r; endmethod
endmodule

// 刁难型 TL-UL 完成方：a_ready 恒高，d_valid 与答复都从同一拍的 A 通道组合出来。
// 0x0C 拒绝；0x10 的读不拒但标坏数据
module mkPickyTl(TlulSlavePins#(8, 32, 4));
  Wire#(Tuple2#(Bool, TlA#(8, 32, 4))) aIn <- mkBypassWire;
  Wire#(Bool)                          dIn <- mkBypassWire;
  Reg#(Bit#(32))                       val <- mkReg(0);
  Reg#(Bit#(16))                       wc  <- mkReg(0);

  function Bool deniedAt(TlA#(8, 32, 4) a) =
    a.address == 8'h0C || !(a.op == opGet || a.op == opPutFull || a.op == opPutPartial);

  function TlD#(32, 4) now();
    match {.v, .a} = aIn;
    TlD#(32, 4) d = answer(a, deniedAt(a), a.address == 8'h08 ? zeroExtend(wc) : val);
    if (a.op == opGet && a.address == 8'h10) d.corrupt = True;
    return d;
  endfunction

  rule commit;
    match {.v, .a} = aIn;
    if (v && dIn && !deniedAt(a) && a.op != opGet) begin
      wc <= wc + 1;
      if (a.address == 8'h04) val <= applyStrb(val, a.data, a.mask);
    end
  endrule

  method Action a_in(Bool av, Bit#(3) op, Bit#(3) param, Bit#(2) size, Bit#(4) source,
                     Bit#(8) address, Bit#(4) mask, Bit#(32) data, Bool corrupt);
    aIn <= tuple2(av, TlA { op: op, size: size, source: source, address: address, mask: mask,
                            data: data, corrupt: corrupt });
  endmethod
  method Bool     a_ready   = True;
  method Bool     d_valid   = tpl_1(aIn);
  method Bit#(3)  d_opcode  = now().opcode;
  method Bit#(2)  d_param   = 0;
  method Bit#(2)  d_size    = now().size;
  method Bit#(4)  d_source  = now().source;
  method Bit#(1)  d_sink    = 0;
  method Bool     d_denied  = now().denied;
  method Bit#(32) d_data    = now().data;
  method Bool     d_corrupt = now().corrupt;
  method Action d_in(Bool dr); dIn <= dr; endmethod
endmodule

(* synthesize *)
module mkBridgeTb(Empty);
  Apb4ToAxi4Lite#(8, 32) b0 <- mkApb4ToAxi4Lite;
  Apb4ToAxi4Lite#(8, 32) b1 <- mkApb4ToAxi4Lite;
  Apb4ToTlul#(8, 32, 4)  b2 <- mkApb4ToTlul;
  Apb4ToTlul#(8, 32, 4)  b3 <- mkApb4ToTlul;

  RegTarget#(8, 32)         dev0 <- mkSlowDev(3);
  Axi4LiteSlavePins#(8, 32) s0   <- mkAxi4LiteBindT(dev0);
  Axi4LiteSlavePins#(8, 32) s1   <- mkPickyAxi;
  RegTarget#(8, 32)         dev2 <- mkSlowDev(3);
  TlulSlavePins#(8, 32, 4)  s2   <- mkTlulBindT(dev2);
  TlulSlavePins#(8, 32, 4)  s3   <- mkPickyTl;

  // ---- 发起方引脚接完成方引脚：往下与往回分两条规则，完成方的答复可能是从请求组合出来的 ----
  function Action axiDown(Axi4LiteMasterPins#(8, 32) m, Axi4LiteSlavePins#(8, 32) s) = action
    s.aw_in(m.awvalid, m.awaddr, m.awprot);
    s.w_in(m.wvalid, m.wdata, m.wstrb);
    s.b_in(m.bready);
    s.ar_in(m.arvalid, m.araddr, m.arprot);
    s.r_in(m.rready);
  endaction;
  function Action axiUp(Axi4LiteMasterPins#(8, 32) m, Axi4LiteSlavePins#(8, 32) s) = action
    m.aw_ready(s.awready);
    m.w_ready(s.wready);
    m.b_rsp(s.bvalid, s.bresp);
    m.ar_ready(s.arready);
    m.r_rsp(s.rvalid, s.rdata, s.rresp);
  endaction;
  function Action tlDown(TlulMasterPins#(8, 32, 4) m, TlulSlavePins#(8, 32, 4) s) = action
    s.a_in(m.a_valid, m.a_opcode, m.a_param, m.a_size, m.a_source, m.a_address, m.a_mask, m.a_data, m.a_corrupt);
    s.d_in(m.d_ready);
  endaction;
  function Action tlUp(TlulMasterPins#(8, 32, 4) m, TlulSlavePins#(8, 32, 4) s) = action
    m.a_rdy(s.a_ready);
    m.d_rsp(s.d_valid, s.d_opcode, s.d_param, s.d_size, s.d_source, s.d_sink, s.d_denied, s.d_data, s.d_corrupt);
  endaction;

  rule down0; axiDown(b0.axi, s0); endrule
  rule up0;   axiUp(b0.axi, s0);   endrule
  rule down1; axiDown(b1.axi, s1); endrule
  rule up1;   axiUp(b1.axi, s1);   endrule
  rule down2; tlDown(b2.tl, s2);   endrule
  rule up2;   tlUp(b2.tl, s2);     endrule
  rule down3; tlDown(b3.tl, s3);   endrule
  rule up3;   tlUp(b3.tl, s3);     endrule

  // ---- APB4 主机：SETUP 一拍、ACCESS 等 PREADY（IHI 0024D 3.1）；命令序列发令牌、主机做完回令牌 ----
  Reg#(UInt#(2))  sel     <- mkConfigReg(0);
  Reg#(Bool)      hw      <- mkConfigReg(False);
  Reg#(Bit#(8))   ha      <- mkConfigReg(0);
  Reg#(Bit#(32))  hd      <- mkConfigReg(0);
  Reg#(Bit#(4))   hs      <- mkConfigReg(0);
  Reg#(UInt#(8))  goTok   <- mkConfigReg(0);
  Reg#(UInt#(8))  doneTok <- mkConfigReg(0);
  Reg#(UInt#(2))  hst     <- mkConfigReg(0);
  Reg#(Bit#(32))  got     <- mkConfigReg(0);
  Reg#(Bool)      gotErr  <- mkConfigReg(False);

  Vector#(4, Apb4SlavePins#(8, 32)) apbs = cons(b0.apb, cons(b1.apb, cons(b2.apb, cons(b3.apb, nil))));

  rule host;
    Bool     pready  = apbs[sel].pready;
    Bit#(32) prdata  = apbs[sel].prdata;
    Bool     pslverr = apbs[sel].pslverr;
    for (Integer i = 0; i < 4; i = i + 1) begin
      Bool me = sel == fromInteger(i);
      apbs[i].req(ha, 3'b000, me && hst != 0, me && hst == 2, hw, hd, hs);
    end
    if (hst == 0 && goTok != doneTok) hst <= 1;
    else if (hst == 1) hst <= 2;
    else if (hst == 2 && pready) begin
      hst <= 0; got <= prdata; gotErr <= pslverr; doneTok <= goTok;
    end
  endrule

  // ---- 下游握手监视：每座桥一条规则，各记各的上一拍 ----
  Vector#(2, Reg#(Bool))     axBad  <- replicateM(mkConfigReg(False));
  Vector#(2, Reg#(Bool))     pAwS   <- replicateM(mkReg(False));
  Vector#(2, Reg#(Bool))     pAwV   <- replicateM(mkReg(False));
  Vector#(2, Reg#(Bit#(8)))  pAwA   <- replicateM(mkReg(0));
  Vector#(2, Reg#(Bool))     pWS    <- replicateM(mkReg(False));
  Vector#(2, Reg#(Bit#(36))) pWD    <- replicateM(mkReg(0));
  Vector#(2, Reg#(Bool))     pArS   <- replicateM(mkReg(False));
  Vector#(2, Reg#(Bit#(8)))  pArA   <- replicateM(mkReg(0));

  function Action axiMon(Integer k, Axi4LiteMasterPins#(8, 32) m, Axi4LiteSlavePins#(8, 32) s) = action
    Bool wrong = False;
    if (pAwS[k] && (!m.awvalid || m.awaddr != pAwA[k])) begin
      $display("FAIL bridge %0d: AWVALID dropped or AWADDR changed before AWREADY (A3.2.1)", k); wrong = True;
    end
    if (pWS[k] && (!m.wvalid || {m.wstrb, m.wdata} != pWD[k])) begin
      $display("FAIL bridge %0d: WVALID dropped or WDATA/WSTRB changed before WREADY (A3.2.1)", k); wrong = True;
    end
    if (pArS[k] && (!m.arvalid || m.araddr != pArA[k])) begin
      $display("FAIL bridge %0d: ARVALID dropped or ARADDR changed before ARREADY (A3.2.1)", k); wrong = True;
    end
    if (m.awvalid && !pAwV[k] && !m.wvalid) begin
      $display("FAIL bridge %0d: AWVALID rose without WVALID; the master must not wait for AWREADY (A3.3.1)", k); wrong = True;
    end
    pAwS[k] <= m.awvalid && !s.awready; pAwV[k] <= m.awvalid; pAwA[k] <= m.awaddr;
    pWS[k]  <= m.wvalid && !s.wready;   pWD[k]  <= {m.wstrb, m.wdata};
    pArS[k] <= m.arvalid && !s.arready; pArA[k] <= m.araddr;
    if (wrong) axBad[k] <= True;
  endaction;

  Vector#(2, Reg#(Bool))      tlBad <- replicateM(mkConfigReg(False));
  Vector#(2, Reg#(Bool))      pAS   <- replicateM(mkReg(False));
  Vector#(2, Reg#(Bit#(54)))  pA    <- replicateM(mkReg(0));
  Reg#(Bit#(3))               expOp <- mkConfigReg(0);

  function Action tlMon(Integer k, TlulMasterPins#(8, 32, 4) m, TlulSlavePins#(8, 32, 4) s) = action
    Bool wrong = False;
    Bit#(54) a = {m.a_opcode, m.a_size, m.a_address, m.a_mask, m.a_data, pack(m.a_corrupt), m.a_source};
    if (pAS[k] && (!m.a_valid || a != pA[k])) begin
      $display("FAIL bridge %0d: a_valid dropped or channel A changed before a_ready (4.1)", k + 2); wrong = True;
    end
    if (m.a_valid && m.a_opcode != expOp) begin
      $display("FAIL bridge %0d: a_opcode %0d, want %0d", k + 2, m.a_opcode, expOp); wrong = True;
    end
    if (m.a_valid && m.a_opcode == opGet && (m.a_mask != 4'hF || m.a_size != 2)) begin
      $display("FAIL bridge %0d: Get with a_mask %h a_size %0d, want f and 2 (7.2)", k + 2, m.a_mask, m.a_size); wrong = True;
    end
    // rocket-chip TLMonitor：monAssert (is_aligned, "'A' channel Get address not aligned to size")，两种 Put 同
    if (m.a_valid && m.a_address[1:0] != 0) begin
      $display("FAIL bridge %0d: a_address %h is not aligned to a_size %0d (TLMonitor is_aligned)", k + 2, m.a_address, m.a_size); wrong = True;
    end
    pAS[k] <= m.a_valid && !s.a_ready; pA[k] <= a;
    if (wrong) tlBad[k] <= True;
  endaction;

  rule mon0; axiMon(0, b0.axi, s0); endrule
  rule mon1; axiMon(1, b1.axi, s1); endrule
  rule mon2; tlMon(0, b2.tl, s2); endrule
  rule mon3; tlMon(1, b3.tl, s3); endrule

  // ---- 命令序列 ----
  Reg#(Bool)      bad <- mkReg(False);
  Reg#(UInt#(32)) cyc <- mkReg(0);

  function Stmt xfer(UInt#(2) b, Bool w, Bit#(8) a, Bit#(32) d, Bit#(4) s, Bit#(3) op) = seq
    action sel <= b; hw <= w; ha <= a; hd <= d; hs <= w ? s : 0; expOp <= op; goTok <= goTok + 1; endaction
    await(doneTok == goTok);
  endseq;

  function Action want(UInt#(2) b, Bit#(32) v, String what) = action
    if (gotErr || got != v) begin
      $display("FAIL bridge %0d, %s: PRDATA %08h PSLVERR %0d, want %08h and 0", b, what, got, gotErr, v); bad <= True;
    end
  endaction;

  function Action wantOk(UInt#(2) b, String what) = action
    if (gotErr) begin $display("FAIL bridge %0d, %s: PSLVERR", b, what); bad <= True; end
  endaction;

  function Action wantErr(UInt#(2) b, String what) = action
    if (!gotErr) begin $display("FAIL bridge %0d, %s: no PSLVERR", b, what); bad <= True; end
  endaction;

  function Stmt suite(UInt#(2) b, Bool picky, Bool axi) = seq
    xfer(b, True,  8'h04, 32'h11111111, 4'hF,    opPutFull);    wantOk(b, "a full write");
    xfer(b, False, 8'h04, 0,            0,       opGet);        want(b, 32'h11111111, "the read after it");
    xfer(b, True,  8'h04, 32'hAABBCCDD, 4'b0101, opPutPartial); wantOk(b, "a write strobing bytes 0 and 2");
    xfer(b, False, 8'h04, 0,            0,       opGet);        want(b, 32'h11BB11DD, "the read after the half-strobed write");
    xfer(b, True,  8'h0C, 32'h1,        4'hF,    opPutFull);    wantErr(b, "a write the target refuses");
    xfer(b, False, 8'h0C, 0,            0,       opGet);        wantErr(b, "a read the target refuses");
    // 刁难型的 0x10：AXI 写读都回 DECERR；TL-UL 写照收，读答 AccessAckData 带 d_corrupt
    if (picky && axi) seq
      xfer(b, True,  8'h10, 32'h1, 4'hF, opPutFull); wantErr(b, "a write answered with DECERR");
      xfer(b, False, 8'h10, 0,     0,    opGet);     wantErr(b, "a read answered with DECERR");
    endseq
    if (picky && !axi) seq
      xfer(b, True,  8'h10, 32'h1, 4'hF, opPutFull); wantOk(b, "a write to 0x10");
      xfer(b, False, 8'h10, 0,     0,    opGet);     wantErr(b, "a read answered with d_corrupt");
    endseq
    xfer(b, False, 8'h08, 0, 0, opGet); want(b, (picky && !axi) ? 32'h3 : 32'h2, "the write count");
    // APB4 的 PADDR 可以落在字中间；到 TL-UL 要按字对齐，读 0x09 读到的是 0x08 那个字
    if (!axi) seq
      xfer(b, False, 8'h09, 0, 0, opGet); want(b, picky ? 32'h3 : 32'h2, "a read at the unaligned address 0x09");
    endseq
  endseq;

  Stmt test = seq
    suite(0, False, True);
    suite(1, True,  True);
    suite(2, False, False);
    suite(3, True,  False);
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule count;
    cyc <= cyc + 1;
    if (cyc > 20000) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  rule fin (started && fsm.done);
    Bool any = bad || axBad[0] || axBad[1] || tlBad[0] || tlBad[1];
    if (any) $display("FAILED");
    else $display("PASS bridge: an APB4 host reads and writes AXI4-Lite and TL-UL targets through the bridges, strobes merge, errors reach PSLVERR, a picky AXI4-Lite target that waits for WVALID and a TL-UL target that answers in the same cycle both work, and the downstream handshakes stay stable until accepted");
    $finish(any ? 1 : 0);
  endrule
endmodule

endpackage
