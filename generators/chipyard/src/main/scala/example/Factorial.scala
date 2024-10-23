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

// DOC include start: GCD params
case class FactorialParams(
  address: BigInt = 0x4000,
  width: Int = 32,
  useAXI4: Boolean = false,
  useBlackBox: Boolean = true,
  useHLS: Boolean = false)
// DOC include end: GCD params

// DOC include start: GCD key
case object FactorialKey extends Field[Option[FactorialParams]](None)
// DOC include end: GCD key

class FactorialIO(val w: Int) extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Bool())
  val input_ready = Output(Bool())
  val input_valid = Input(Bool())
  val n = Input(UInt(w.W))
  val output_ready = Input(Bool())
  val output_valid = Output(Bool())
  val factorial = Output(UInt(w.W))
  val busy = Output(Bool())
}

class HLSFactorialAccelIO(val w: Int) extends Bundle {
  val ap_clk = Input(Clock())
  val ap_rst = Input(Reset())
  val ap_start = Input(Bool())
  val ap_done = Output(Bool())
  val ap_idle = Output(Bool())
  val ap_ready = Output(Bool())
  val n = Input(UInt(w.W))
  val ap_return = Output(UInt(w.W))
}

class FactorialTopIO extends Bundle {
  val factorial_busy = Output(Bool())
}

trait HasFactorialTopIO {
  def io: FactorialTopIO
}

// DOC include start: GCD blackbox
class FactorialMMIOBlackBox(val w: Int) extends BlackBox(Map("WIDTH" -> IntParam(w))) with HasBlackBoxResource {
  val io = IO(new FactorialIO(w))
  addResource("/vsrc/FactorialMMIOBlackBox.v")
}
// DOC include end: GCD blackbox

// DOC include start: GCD chisel
class FactorialMMIOChiselModule(val w: Int) extends Module {
  val io = IO(new FactorialIO(w))
  val s_idle :: s_compute :: s_done :: Nil = Enum(3)

  val state = RegInit(s_idle)
  val n     = Reg(UInt(w.W))
  val result = Reg(UInt(w.W))
  val counter = Reg(UInt(w.W))  // Counter to iterate through the multiplication

  io.input_ready := state === s_idle
  io.output_valid := state === s_done
  io.factorial := result

  when (state === s_idle && io.input_valid) {
    state := s_compute
  } .elsewhen (state === s_compute && counter === 1.U) {
    state := s_done
  } .elsewhen (state === s_done && io.output_ready) {
    state := s_idle
  }

  when (state === s_idle && io.input_valid) {
    result := 1.U
    n      := io.n
    counter := io.n  // Initialize counter to the input value
  } .elsewhen (state === s_compute && counter > 1.U) {
    result := result * counter  // Multiply result with current counter
    counter := counter - 1.U    // Decrement counter
  }

  io.busy := state =/= s_idle
}
// DOC include end: GCD chisel

// DOC include start: HLS blackbox
class HLSFactorialAccelBlackBox(val w: Int) extends BlackBox with HasBlackBoxPath {
  val io = IO(new HLSFactorialAccelIO(w))

  val chipyardDir = System.getProperty("user.dir")
  val hlsDir = s"$chipyardDir/generators/chipyard"
  
  // Run HLS command
  val make = s"make -C ${hlsDir}/src/main/resources/hls default"
  require (make.! == 0, "Failed to run HLS")

  // Add each vlog file
  addPath(s"$hlsDir/src/main/resources/vsrc/HLSFactorialAccelBlackBox.v")
  addPath(s"$hlsDir/src/main/resources/vsrc/HLSFactorialAccelBlackBox_flow_control_loop_pipe.v")
}
// DOC include end: HLS blackbox

// DOC include start: GCD router
class FactorialTL(params: FactorialParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("factorial", Seq("ucbbar,factorial")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  override lazy val module = new FactorialImpl
  class FactorialImpl extends Impl with HasFactorialTopIO {
    val io = IO(new FactorialTopIO)
    withClockAndReset(clock, reset) {
      // How many clock cycles in a PWM cycle?
      val n = Reg(UInt(params.width.W))
      val factorial = Wire(new DecoupledIO(UInt(params.width.W)))
      val status = Wire(UInt(2.W))

      val impl_io = if (params.useBlackBox) {
        val impl = Module(new FactorialMMIOBlackBox(params.width))
        impl.io
      } else {
        val impl = Module(new FactorialMMIOChiselModule(params.width))
        impl.io
      }

 
      impl_io.clock := clock
      impl_io.reset := reset.asBool

      impl_io.n := n
      impl_io.input_valid := true.B
      factorial.bits := impl_io.factorial
      factorial.valid := impl_io.output_valid
      impl_io.output_ready := factorial.ready

      status := Cat(impl_io.input_ready, impl_io.output_valid)
      io.factorial_busy := impl_io.busy

// DOC include start: Factorial instance regmap
      node.regmap(
        0x00 -> Seq(
          RegField.r(2, status)), // a read-only register capturing current status
        0x04 -> Seq(
          RegField.w(params.width, n)), // a plain, write-only register for `n`
        0x08 -> Seq(
          RegField.r(params.width, factorial))) // read-only register for factorial result
// DOC include end: Factorial instance regmap
    }
  }
}

// DOC include end: Factorial router
// DOC include start: Factorial AXI4
class FactorialAXI4(params: FactorialParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val node = AXI4RegisterNode(AddressSet(params.address, 4096-1), beatBytes=beatBytes)
  override lazy val module = new FactorialImpl
  class FactorialImpl extends Impl with HasFactorialTopIO {
    val io = IO(new FactorialTopIO)
    withClockAndReset(clock, reset) {
      val n = Reg(UInt(params.width.W)) // Input number `n` for factorial
      val factorial = Wire(new DecoupledIO(UInt(params.width.W))) // Output factorial
      val status = Wire(UInt(2.W))

      val impl_io = if (params.useBlackBox) {
        val impl = Module(new FactorialMMIOBlackBox(params.width)) // Use Factorial BlackBox
        impl.io
      } else {
        val impl = Module(new FactorialMMIOChiselModule(params.width)) // Use Chisel implementation
        impl.io
      }

      impl_io.clock := clock
      impl_io.reset := reset.asBool

      impl_io.n := n
      impl_io.input_valid := true.B // Always valid input for factorial
      factorial.bits := impl_io.factorial
      factorial.valid := impl_io.output_valid
      impl_io.output_ready := factorial.ready

      status := Cat(impl_io.input_ready, impl_io.output_valid)
      io.factorial_busy := impl_io.busy

      node.regmap(
        0x00 -> Seq(
          RegField.r(2, status)), // read-only register capturing status
        0x04 -> Seq(
          RegField.w(params.width, n)), // write-only register for input `n`
        0x08 -> Seq(
          RegField.r(params.width, factorial))) // read-only register for factorial result
    }
  }
}
// DOC include end: GCD router
// DOC include start: HLS Factorial Accel
class HLSFactorialAccel(params: FactorialParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("hlsgcdaccel", Seq("ucbbar,hlsgcdaccel"))
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  override lazy val module = new HLSFactorialAccelImpl
  class HLSFactorialAccelImpl extends Impl with HasFactorialTopIO {
    val io = IO(new FactorialTopIO)
    withClockAndReset(clock, reset) {
      val n = Reg(UInt(params.width.W)) // Input number for factorial
      val factorial = Wire(new DecoupledIO(UInt(params.width.W))) // Output factorial
      val factorial_reg = Reg(UInt(params.width.W))
      val status = Wire(UInt(2.W))

      val impl = Module(new HLSFactorialAccelBlackBox(params.width)) // Factorial HLS BlackBox

      impl.io.ap_clk := clock
      impl.io.ap_rst := reset

      val s_idle :: s_busy :: Nil = Enum(2)
      val state = RegInit(s_idle)
      val result_valid = RegInit(false.B)

      when (state === s_idle && impl.io.ap_idle) {
        state := s_busy
        result_valid := false.B
      } .elsewhen (state === s_busy && impl.io.ap_done) {
        state := s_idle
        result_valid := true.B
        factorial_reg := impl.io.ap_return // Store factorial result
      }

      impl.io.ap_start := state === s_busy

      factorial.valid := result_valid
      status := Cat(impl.io.ap_idle, result_valid)

      impl.io.n := n
      factorial.bits := factorial_reg

      io.factorial_busy := !impl.io.ap_idle

      node.regmap(
        0x00 -> Seq(
          RegField.r(2, status)), // read-only register capturing current status
        0x04 -> Seq(
          RegField.w(params.width, n)), // write-only register for input `n`
        0x08 -> Seq(
          RegField.r(params.width, factorial))) // read-only register for factorial result
    }
  }
}
// DOC include end: HLS Factorial Accel
// DOC include start: Factorial lazy trait
trait CanHavePeripheryFactorial { this: BaseSubsystem =>
  private val portName = "factorial"

  private val pbus = locateTLBusWrapper(PBUS)

  // Only build if we are using the TL (nonAXI4) version
  val factorial_busy = p(FactorialKey) match {
    case Some(params) => {
      val factorial = if (params.useAXI4) {
        val factorial = LazyModule(new FactorialAXI4(params, pbus.beatBytes)(p))
        factorial.clockNode := pbus.fixedClockNode
        pbus.coupleTo(portName) {
          factorial.node := 
          AXI4Buffer() := 
          TLToAXI4() := 
          TLFragmenter(pbus.beatBytes, pbus.blockBytes, holdFirstDeny = true) := _
        }
        factorial
      } else if (params.useHLS) {
        val factorial = LazyModule(new HLSFactorialAccel(params, pbus.beatBytes)(p))
        factorial.clockNode := pbus.fixedClockNode
        pbus.coupleTo(portName) { factorial.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }
        factorial
      } else {
        val factorial = LazyModule(new FactorialTL(params, pbus.beatBytes)(p))
        factorial.clockNode := pbus.fixedClockNode
        pbus.coupleTo(portName) { factorial.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }
        factorial
      }
      val factorial_busy = InModuleBody {
        val busy = IO(Output(Bool())).suggestName("factorial_busy")
        busy := factorial.module.io.factorial_busy
        busy
      }
      Some(factorial_busy)
    }
    case None => None
  }
}
// DOC include end: Factorial lazy trait
// DOC include start: Factorial config fragment
class WithFactorial(useAXI4: Boolean = false, useBlackBox: Boolean = false, useHLS: Boolean = false) extends Config((site, here, up) => {
  case FactorialKey => {
    // useHLS cannot be used with useAXI4 and useBlackBox
    assert(!useHLS || (useHLS && !useAXI4 && !useBlackBox)) 
    Some(FactorialParams(useAXI4 = useAXI4, useBlackBox = useBlackBox, useHLS = useHLS))
  }
})
// DOC include end: Factorial config fragment
