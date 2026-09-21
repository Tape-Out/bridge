package Bridge;

// 总线之间的桥：上游总线的完成方 + 下游总线的发起方，中间是中立的会停顿目标。桥本身是 hwcore
// 里 Bus 类型类的一个泛型函数，这里只给常用的几对配上出芯片的引脚，做成可以综合的顶层。

import RegIf::*;
import Bus::*;
import Apb4::*;
import Axi4Lite::*;
import Tilelink::*;

interface Apb4ToAxi4Lite#(numeric type aw, numeric type dw);
  interface Apb4SlavePins#(aw, dw)      apb;
  interface Axi4LiteMasterPins#(aw, dw) axi;
endinterface

// 类型类的方法定死在 Module 上，调它的模块也要写明 [Module]，否则 bsc 推成 IsModule 多态报 T0029
module [Module] mkApb4ToAxi4Lite(Apb4ToAxi4Lite#(aw, dw));
  Axi4LiteWire#(aw, dw)  w  <- mkAxi4LiteWire;
  Apb4SlavePins#(aw, dw) up <- bridge(w.slave);
  interface apb = up;
  interface axi = w.master;
endmodule

interface Apb4ToTlul#(numeric type aw, numeric type dw, numeric type sw);
  interface Apb4SlavePins#(aw, dw)      apb;
  interface TlulMasterPins#(aw, dw, sw) tl;
endinterface

module [Module] mkApb4ToTlul(Apb4ToTlul#(aw, dw, sw))
    provisos (Mul#(TDiv#(dw, 8), 8, dw));
  TlulWire#(aw, dw, sw)  w  <- mkTlulWire;
  Apb4SlavePins#(aw, dw) up <- bridge(w.slave);
  interface apb = up;
  interface tl  = w.master;
endmodule

endpackage
