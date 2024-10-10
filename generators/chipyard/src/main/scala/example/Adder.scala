package chipyard.example

import sys.process._

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, BaseModule}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.UIntIsOneOf

// DOC include start: Adder params
case class AdderParams(
  address: BigInt = 0x4000,
  width: Int = 32,
  useAXI4: Boolean = false,
  useBlackBox: Boolean = true,
  useHLS: Boolean = false)
// DOC include end: Adder params

// DOC include start: Adder key
case object AdderKey extends Field[Option[AdderParams]](None)
// DOC include end: Adder key

class AdderIO(val w: Int) extends Bundle {
  // val clock = Input(Clock())
  // val reset = Input(Bool())
  // val input_ready = Output(Bool())
  // val input_valid = Input(Bool())
  val x = Input(UInt(w.W))
  val y = Input(UInt(w.W))
  // val output_ready = Input(Bool())
  // val output_valid = Output(Bool())
  val adder = Output(UInt(w.W))
  //val busy = Output(Bool())
}

class HLSAdderAccelIO(val w: Int) extends Bundle {
  val ap_clk = Input(Clock())
  val ap_rst = Input(Reset())
  val ap_start = Input(Bool())
  val ap_done = Output(Bool())
  val ap_idle = Output(Bool())
  val ap_ready = Output(Bool())
  val x = Input(UInt(w.W))
  val y = Input(UInt(w.W))
  val ap_return = Output(UInt(w.W))
}

class AdderTopIO extends Bundle {
  val adder_busy = Output(Bool())
}

trait HasAdderTopIO {
  def io: AdderTopIO
}

// DOC include start: Adder blackbox
class AdderMMIOBlackBox(val w: Int) extends BlackBox(Map("WIDTH" -> IntParam(w))) with HasBlackBoxResource {
  val io = IO(new AdderIO(w))
  addResource("/vsrc/AdderMMIOBlackBox.v")
}
// DOC include end: Adder blackbox

// DOC include start: Adder chisel
class AdderMMIOChiselModule(val w: Int) extends Module {
  val io = IO(new AdderIO(w))
  val s_idle :: s_run :: s_done :: Nil = Enum(3)

  val state = RegInit(s_idle)
  val tmp   = Reg(UInt(w.W))
  val adder   = Reg(UInt(w.W))

  io.input_ready := state === s_idle
  io.output_valid := state === s_done
  io.adder := adder

  when (state === s_idle && io.input_valid) {
    state := s_run
  } .elsewhen (state === s_run && tmp === 0.U) {
    state := s_done
  } .elsewhen (state === s_done && io.output_ready) {
    state := s_idle
  }

  when (state === s_idle && io.input_valid) {
    adder := io.x
    tmp := io.y
  } .elsewhen (state === s_run) {
    when (adder > tmp) {
      adder := adder - tmp
    } .otherwise {
      tmp := tmp - adder
    }
  }

  io.busy := state =/= s_idle
}
// DOC include end: Adder chisel

// DOC include start: HLS blackbox
class HLSAdderAccelBlackBox(val w: Int) extends BlackBox with HasBlackBoxPath {
  val io = IO(new HLSAdderAccelIO(w))

  val chipyardDir = System.getProperty("user.dir")
  val hlsDir = s"$chipyardDir/generators/chipyard"
  
  // Run HLS command
  val make = s"make -C ${hlsDir}/src/main/resources/hls default"
  require (make.! == 0, "Failed to run HLS")

  // Add each vlog file
  addPath(s"$hlsDir/src/main/resources/vsrc/HLSAdderAccelBlackBox.v")
  addPath(s"$hlsDir/src/main/resources/vsrc/HLSAdderAccelBlackBox_flow_control_loop_pipe.v")
}
// DOC include end: HLS blackbox

// DOC include start: Adder router
class AdderTL(params: AdderParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("adder", Seq("ucbbar,adder")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  override lazy val module = new AdderImpl
  class AdderImpl extends Impl with HasAdderTopIO {
    val io = IO(new AdderTopIO)
    withClockAndReset(clock, reset) {
      // How many clock cycles in a PWM cycle?
      val x = Reg(UInt(params.width.W))
      val y = Wire(new DecoupledIO(UInt(params.width.W)))
      val adder = Wire(new DecoupledIO(UInt(params.width.W)))
      val status = Wire(UInt(2.W))

      val impl_io = if (params.useBlackBox) {
        val impl = Module(new AdderMMIOBlackBox(params.width))
        impl.io
      } else {
        val impl = Module(new AdderMMIOChiselModule(params.width))
        impl.io
      }

      impl_io.clock := clock
      impl_io.reset := reset.asBool

      impl_io.x := x
      impl_io.y := y.bits
      impl_io.input_valid := y.valid
      y.ready := impl_io.input_ready

      adder.bits := impl_io.adder
      adder.valid := impl_io.output_valid
      impl_io.output_ready := adder.ready

      status := Cat(impl_io.input_ready, impl_io.output_valid)
      io.adder_busy := impl_io.busy

// DOC include start: Adder instance regmap
      node.regmap(
        0x00 -> Seq(
          RegField.r(2, status)), // a read-only register capturing current status
        0x04 -> Seq(
          RegField.w(params.width, x)), // a plain, write-only register
        0x08 -> Seq(
          RegField.w(params.width, y)), // write-only, y.valid is set on write
        0x0C -> Seq(
          RegField.r(params.width, adder))) // read-only, adder.ready is set on read
// DOC include end: Adder instance regmap
    }
  }
}

class AdderAXI4(params: AdderParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val node = AXI4RegisterNode(AddressSet(params.address, 4096-1), beatBytes=beatBytes)
  override lazy val module = new AdderImpl
  class AdderImpl extends Impl with HasAdderTopIO {
    val io = IO(new AdderTopIO)
    withClockAndReset(clock, reset) {
      // How many clock cycles in a PWM cycle?
      val x = Reg(UInt(params.width.W))
      val y = Wire(new DecoupledIO(UInt(params.width.W)))
      val adder = Wire(new DecoupledIO(UInt(params.width.W)))
      val status = Wire(UInt(2.W))

      val impl_io = if (params.useBlackBox) {
        val impl = Module(new AdderMMIOBlackBox(params.width))
        impl.io
      } else {
        val impl = Module(new AdderMMIOChiselModule(params.width))
        impl.io
      }

      impl_io.clock := clock
      impl_io.reset := reset.asBool

      impl_io.x := x
      impl_io.y := y.bits
      impl_io.input_valid := y.valid
      y.ready := impl_io.input_ready

      adder.bits := impl_io.adder
      adder.valid := impl_io.output_valid
      impl_io.output_ready := adder.ready

      status := Cat(impl_io.input_ready, impl_io.output_valid)
      io.adder_busy := impl_io.busy

      node.regmap(
        0x00 -> Seq(
          RegField.r(2, status)), // a read-only register capturing current status
        0x04 -> Seq(
          RegField.w(params.width, x)), // a plain, write-only register
        0x08 -> Seq(
          RegField.w(params.width, y)), // write-only, y.valid is set on write
        0x0C -> Seq(
          RegField.r(params.width, adder))) // read-only, adder.ready is set on read
    }
  }
}
// DOC include end: Adder router

class HLSAdderAccel(params: AdderParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("hlsadderaccel", Seq("ucbbar,hlsadderaccel")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  override lazy val module = new HLSAdderAccelImpl
  class HLSAdderAccelImpl extends Impl with HasAdderTopIO {
    val io = IO(new AdderTopIO)
    withClockAndReset(clock, reset) {
      val x = Reg(UInt(params.width.W))
      val y = Wire(new DecoupledIO(UInt(params.width.W)))
      val y_reg = Reg(UInt(params.width.W))
      val adder = Wire(new DecoupledIO(UInt(params.width.W)))
      val adder_reg = Reg(UInt(params.width.W))
      val status = Wire(UInt(2.W))

      val impl = Module(new HLSAdderAccelBlackBox(params.width))

      impl.io.ap_clk := clock
      impl.io.ap_rst := reset

      val s_idle :: s_busy :: Nil = Enum(2)
      val state = RegInit(s_idle)
      val result_valid = RegInit(false.B)
      when (state === s_idle && y.valid) { 
        state := s_busy
        result_valid := false.B
        y_reg := y.bits 
      } .elsewhen (state === s_busy && impl.io.ap_done) {
        state := s_idle
        result_valid := true.B
        adder_reg := impl.io.ap_return
      }

      impl.io.ap_start := state === s_busy

      adder.valid := result_valid
      status := Cat(impl.io.ap_idle, result_valid)
      
      impl.io.x := x
      impl.io.y := y_reg
      y.ready := impl.io.ap_idle
      adder.bits := adder_reg

      io.adder_busy := !impl.io.ap_idle

      node.regmap(
        0x00 -> Seq(
          RegField.r(2, status)), // a read-only register capturing current status
        0x04 -> Seq(
          RegField.w(params.width, x)), // a plain, write-only register
        0x08 -> Seq(
          RegField.w(params.width, y)), // write-only, y.valid is set on write
        0x0C -> Seq(
          RegField.r(params.width, adder))) // read-only, adder.ready is set on read
    }
  }
}

// DOC include start: Adder lazy trait
trait CanHavePeripheryAdder { this: BaseSubsystem =>
  private val portName = "adder"

  private val pbus = locateTLBusWrapper(PBUS)

  // Only build if we are using the TL (nonAXI4) version
  val adder_busy = p(AdderKey) match {
    case Some(params) => {
      val adder = if (params.useAXI4) {
        val adder = LazyModule(new AdderAXI4(params, pbus.beatBytes)(p))
        adder.clockNode := pbus.fixedClockNode
        pbus.coupleTo(portName) {
          adder.node :=
          AXI4Buffer () :=
          TLToAXI4 () :=
          // toVariableWidthSlave doesn't use holdFirstDeny, which TLToAXI4() needsx
          TLFragmenter(pbus.beatBytes, pbus.blockBytes, holdFirstDeny = true) := _
        }
        adder
      } else if (params.useHLS) {
        val adder = LazyModule(new HLSAdderAccel(params, pbus.beatBytes)(p))
        adder.clockNode := pbus.fixedClockNode
        pbus.coupleTo(portName) { adder.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }
        adder
      } else {
        val adder = LazyModule(new AdderTL(params, pbus.beatBytes)(p))
        adder.clockNode := pbus.fixedClockNode
        pbus.coupleTo(portName) { adder.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }
        adder
      }
      val adder_busy = InModuleBody {
        val busy = IO(Output(Bool())).suggestName("adder_busy")
        busy := adder.module.io.adder_busy
        busy
      }
      Some(adder_busy)
    }
    case None => None
  }
}
// DOC include end: Adder lazy trait

// DOC include start: Adder config fragment
class WithAdder(useAXI4: Boolean = false, useBlackBox: Boolean = false, useHLS: Boolean = false) extends Config((site, here, up) => {
  case AdderKey => {
    // useHLS cannot be used with useAXI4 and useBlackBox
    assert(!useHLS || (useHLS && !useAXI4 && !useBlackBox)) 
    Some(AdderParams(useAXI4 = useAXI4, useBlackBox = useBlackBox, useHLS = useHLS))
  }
})
// DOC include end: Adder config fragment
