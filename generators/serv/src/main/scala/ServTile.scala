
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// SERV Tile Wrapper
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package serv

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, StringParam, RawParam}


import org.chipsalliance.cde.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.rocket._
import freechips.rocketchip.subsystem.{RocketCrossingParams}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import freechips.rocketchip.prci._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.resources.SimpleDevice
import freechips.rocketchip.resources.ResourceAnchors
import freechips.rocketchip.resources.ResourceBindings
import freechips.rocketchip.resources.ResourceBinding
import freechips.rocketchip.resources.Description
import freechips.rocketchip.resources.{Resource, ResourceAddress}


case class ServCoreParams(
  val bootFreqHz: BigInt = BigInt(32000000),
  val xLen: Int = 32,
  val pmpEnable: Int = 0,
  val pmpGranularity: Int = 0,
  val pmpNumRegions: Int = 0
) extends CoreParams {
  val useVM: Boolean = false
  val useHypervisor: Boolean = false
  val useUser: Boolean = true
  val useSupervisor: Boolean = false
  val useDebug: Boolean = false
  val useAtomics: Boolean = false
  val useAtomicsOnlyForIO: Boolean = false
  val useCompressed: Boolean = false
  override val useVector: Boolean = false
  val useSCIE: Boolean = false
  val useRVE: Boolean = false
  val mulDiv: Option[MulDivParams]= None
  val fpu: Option[FPUParams] = None //floating point not supported
  val fetchWidth: Int = 1
  val decodeWidth: Int = 1
  val retireWidth: Int = 1
  val instBits: Int =  32
  val nLocalInterrupts: Int = 1
  val nPMPs: Int = 0
  val nBreakpoints: Int = 0
  val useBPWatch: Boolean = false
  val nPerfCounters: Int = 0
  val haveBasicCounters: Boolean = true
  val haveFSDirty: Boolean = false
  val misaWritable: Boolean = false
  val haveCFlush: Boolean = false
  val nL2TLBEntries: Int = 0
  val mtvecInit: Option[BigInt] = Some(0)
  val mtvecWritable: Boolean = false
  val nL2TLBWays: Int = 0
  val lrscCycles: Int = 80
  val useNMI = false                  // SERV does not support NMI
  val useConditionalZero = false     // No special optimization for zero
  val useZba = false                 // Bit-manipulation extension not supported
  val useZbb = false
  val useZbs = false
  val traceHasWdata = false          // No tracing with write data
  val pgLevels = 0                   // No page tables (no virtual memory)
  val nPTECacheEntries = 0           // No PTE cache
  val mcontextWidth = 0              // No machine context
  val scontextWidth = 0              // No supervisor context
  // SERV SPECIFIC
  val aw_b: Int = 12
  val iw_b: Int = 0
  val uw_b: Int = 0
  val with_csr_b: Boolean = true
  val memsize: Int = 8192
  val sim_b: Int = 1
  val memfile_b: String = ""
  val reset_strategy: String = "MINI"
}


case class ServTileAttachParams(
  tileParams: ServTileParams,
  crossingParams: RocketCrossingParams
) extends CanAttachTile {
  type TileType = ServTile
  val lookup = PriorityMuxHartIdFromSeq(Seq(tileParams))
}

case class ServTileParams(
  name: Option[String] = Some("serv_tile"),
  tileId: Int = 0,
  val core: ServCoreParams = ServCoreParams()
) extends InstantiableTileParams[ServTile]
{
  val beuAddr: Option[BigInt] = None
  val blockerCtrlAddr: Option[BigInt] = None
  val btb: Option[BTBParams] = None
  val boundaryBuffers: Boolean = false
  val dcache: Option[DCacheParams] = None //no dcache
  val icache: Option[ICacheParams] = None //no icache -- Bit serial processing
  val clockSinkParams: ClockSinkParameters = ClockSinkParameters()
  def instantiate(crossing: HierarchicalElementCrossingParamsLike, lookup: LookupByHartIdImpl)(implicit p: Parameters): ServTile = {
    new ServTile(this, crossing, lookup)
  }

  val baseName = name.getOrElse("serv_tile")
  val uniqueName = s"${baseName}_$tileId"
} 

class ServTile private(
  val servParams: ServTileParams,
  crossing: ClockCrossingType,
  lookup: LookupByHartIdImpl,
  q: Parameters)
  extends BaseTile(servParams, crossing, lookup, q)
  with SinksExternalInterrupts
  with SourcesExternalNotifications
{

  def this(params: ServTileParams, crossing: HierarchicalElementCrossingParamsLike, lookup: LookupByHartIdImpl)(implicit p: Parameters) =
    this(params, crossing.crossingType, lookup, p)

  //TL nodes
  val intOutwardNode = None   
  val masterNode = visibilityNode         // Master interface — TL visibility
  val slaveNode  = TLIdentityNode()        // Slave node for MMIO

  val beatBytes = p(PeripheryBusKey).beatBytes

  tlOtherMastersNode := tlMasterXbar.node
  masterNode :=* tlOtherMastersNode
  DisableMonitors { implicit p => tlSlaveXbar.node :*= slaveNode }
  
  // Required entry of CPU device in the device tree for interrupt purpose
  val cpuDevice: SimpleDevice = new SimpleDevice("cpu", Seq("OlofKindgren,serv", "riscv")) {
    override def parent = Some(ResourceAnchors.cpus)
    override def describe(resources: ResourceBindings): Description = {
      val Description(name, mapping) = super.describe(resources)
      Description(name, mapping ++
                        cpuProperties ++
                        nextLevelCacheProperty ++
                        tileProperties)
    }
  }

  ResourceBinding {
    Resource(cpuDevice, "reg").bind(ResourceAddress(tileId))
  }


override def makeMasterBoundaryBuffers(crossing: ClockCrossingType)(implicit p: Parameters) = crossing match {
    case _: RationalCrossing =>
      if (!servParams.boundaryBuffers) TLBuffer(BufferParams.none)
      else TLBuffer(BufferParams.none, BufferParams.flow, BufferParams.none, BufferParams.flow, BufferParams(1))
    case _ => TLBuffer(BufferParams.none)
  }

override def makeSlaveBoundaryBuffers(crossing: ClockCrossingType)(implicit p: Parameters) = crossing match {
    case _: RationalCrossing =>
      if (!servParams.boundaryBuffers) TLBuffer(BufferParams.none)
      else TLBuffer(BufferParams.flow, BufferParams.none, BufferParams.none, BufferParams.none, BufferParams.none)
    case _ => TLBuffer(BufferParams.none)
  }



override lazy val module = new ServTileModuleImp(this)


  val portNameM = "serv-axi4-master"
  val idBitsM = 0
  //val beatBytes = 4

  val ServAXI4MNode = AXI4MasterNode(
    Seq(AXI4MasterPortParameters(
      masters = Seq(AXI4MasterParameters(
        name = portNameM,
        id = IdRange(0, 1 << idBitsM))))))

  val memoryTap = TLIdentityNode()
 (tlMasterXbar.node  
    := memoryTap
    := TLBuffer()
    := TLFIFOFixer(TLFIFOFixer.all) // fix FIFO ordering
    := TLWidthWidget(masterPortBeatBytes) // reduce size of TL
    := AXI4ToTL() // convert to TL
    := AXI4UserYanker(Some(2)) // remove user field on AXI interface. need but in reality user intf. not needed
    := AXI4Fragmenter() // deal with multi-beat xacts
    := ServAXI4MNode) // Custom SERV node.



/*val slaveTLNode = TLIdentityNode()

val ServAXI4SNode = AXI4SlaveNode(Seq(
  AXI4SlavePortParameters(
    slaves = Seq(AXI4SlaveParameters(
      address       = Seq(AddressSet(0x40000000L, 0x3FF)),
      resources     = (new SimpleDevice("serv", Seq("ucbbar,serv"))).reg("mem"),
      executable    = false,
      supportsRead  = TransferSizes(1, beatBytes),
      supportsWrite = TransferSizes(1, beatBytes)
    )),
    beatBytes = beatBytes
  )
))*/

/* Connect TileLink side to AXI4 side
ServAXI4SNode :=
  AXI4Fragmenter() := AXI4UserYanker() := AXI4Deinterleaver(beatBytes) :=
  TLToAXI4() := TLBuffer() := TLWidthWidget(beatBytes) := slaveTLNode*/

// Directly attach the SERV slave TL node to the PeripheryBus (for internal access)
//pbuss.node := TLBuffer() := slaveTLNode

def connectServInterrupts(mtip: Bool): Unit = {
    val (interrupts, _) = intSinkNode.in(0)
    mtip := interrupts(1)
  }
}
  
class ServTileModuleImp(outer: ServTile) extends BaseTileModuleImp(outer){
  

  val core = Module(new ServCoreBlackbox(
  memfile_b        = outer.servParams.core.memfile_b,
  memsize_b        = outer.servParams.core.memsize,
  sim_b            = outer.servParams.core.sim_b ,
  reset_strategy_b = outer.servParams.core.reset_strategy,
  with_csr_b       = if(outer.servParams.core.with_csr_b) 1 else 0,
  aw_b             = outer.servParams.core.aw_b,
  uw_b             = outer.servParams.core.uw_b,
  iw_b             = outer.servParams.core.iw_b
))


  core.io.clk := clock
  core.io.rst := reset.asBool    // Check Reset --
  

  outer.connectServInterrupts(core.io.i_timer_irq)

//------------------SERV MASTER NODE CONNCETION WITH AXI BUNDLE-----------//
//--------------------FROM SERVING TO EXTERNAL
  // connect the axi interface
  outer.ServAXI4MNode.out foreach { case (out, edgeOut) =>
    
    core.io.i_awmready             := out.aw.ready
    out.aw.valid                   := core.io.o_awmvalid
    out.aw.bits.addr               := core.io.o_awmaddr
    // unused signals
    assert (core.io.o_awm_id  === 0.U) 
    assert (core.io.o_awm_len  === 0.U)
    assert (core.io.o_awm_size  === 0.U)
    assert (core.io.o_awm_burst === 0.U)
    assert (core.io.o_awm_lock  === 0.U)
    assert (core.io.o_awm_cache === 0.U)
    assert (core.io.o_awm_prot  === 0.U)
    assert (core.io.o_awm_qos   === 0.U)
    assert (core.io.o_awm_region === 0.U)
    assert (core.io.o_awm_atop  === 0.U)
    assert (core.io.o_awm_user  === 0.U)  

    core.io.i_wmready              := out.w.ready
    out.w.valid                    := core.io.o_wmvalid
    out.w.bits.data                := core.io.o_wmdata
    out.w.bits.strb                := core.io.o_wmstrb   
    // unused signals
     assert(core.io.o_wm_user === 0.U)
     assert(core.io.o_wm_last === 0.U)       

    out.b.ready                    := core.io.o_bmready
    core.io.i_bmvalid              := out.b.valid
    core.io.i_bmresp               := out.b.bits.resp
    // unused signals
    core.io.i_bm_id  := 0.U    
    core.io.i_bm_user  := 0.U 
    

    core.io.i_armready             := out.ar.ready
    out.ar.valid                   := core.io.o_armvalid
    out.ar.bits.addr               := core.io.o_armaddr
    // unused signals 
    assert (core.io.o_arm_id  === 0.U) 
    assert (core.io.o_arm_len  === 0.U)
    assert (core.io.o_arm_size  === 0.U)
    assert (core.io.o_arm_burst === 0.U)
    assert (core.io.o_arm_lock  === 0.U)
    assert (core.io.o_arm_cache === 0.U)
    assert (core.io.o_arm_prot  === 0.U)
    assert (core.io.o_arm_qos   === 0.U)
    assert (core.io.o_arm_region === 0.U)
    assert (core.io.o_arm_user  === 0.U) 
    
    out.r.ready                    := core.io.o_rmready
    core.io.i_rmvalid              := out.r.valid
    core.io.i_rmdata               := out.r.bits.data
    core.io.i_rmresp               := out.r.bits.resp
    core.io.i_rmlast               := out.r.bits.last
    //unused signals
    core.io.i_rm_user := 0.U 
    core.io.i_rm_id := 0.U 
  }
            
  /*------------SERV SLAVE NODE CONNECTION WITH AXI BUNDLE-----------------//
  //-------------FROM EXTERNAL TO SERVING
outer.ServAXI4SNode.in foreach { case (in, edgeIn) =>
  in.aw.ready := core.io.o_awready
  core.io.i_awvalid := in.aw.valid
  core.io.i_awaddr  := in.aw.bits.addr
  //unused signals
  core.io.i_aw_id     := 0.U
  core.io.i_aw_len    := 0.U
  core.io.i_aw_size   := 0.U
  core.io.i_aw_burst  := 0.U
  core.io.i_aw_lock   := 0.U
  core.io.i_aw_cache  := 0.U
  core.io.i_aw_prot   := 0.U
  core.io.i_aw_qos    := 0.U
  core.io.i_aw_region := 0.U
  core.io.i_aw_atop   := 0.U
  core.io.i_aw_user   := 0.U

  

  in.w.ready := core.io.o_wready
  core.io.i_wvalid := in.w.valid
  core.io.i_wdata  := in.w.bits.data
  core.io.i_wstrb  := in.w.bits.strb
  //unused signals
  core.io.i_w_last := 0.U
  core.io.i_w_user := 0.U

  
  core.io.i_bready := in.b.ready
  in.b.valid := core.io.o_bvalid
  in.b.bits.resp := core.io.o_bresp
  //unused signals
  assert (core.io.o_b_id   === 0.U)
  assert (core.io.o_b_user === 0.U)

  in.ar.ready := core.io.o_arready
  core.io.i_arvalid := in.ar.valid
  core.io.i_araddr  := in.ar.bits.addr
  //unused signals:
  core.io.i_ar_id     := 0.U
  core.io.i_ar_len    := 0.U
  core.io.i_ar_size   := 0.U
  core.io.i_ar_burst  := 0.U
  core.io.i_ar_lock   := 0.U
  core.io.i_ar_cache  := 0.U
  core.io.i_ar_prot   := 0.U
  core.io.i_ar_qos    := 0.U
  core.io.i_ar_region := 0.U
  core.io.i_ar_user   := 0.U


  core.io.i_rready := in.r.ready
  in.r.valid := core.io.o_rvalid
  in.r.bits.data := core.io.o_rdata
  in.r.bits.resp := core.io.o_rresp
  in.r.bits.last := core.io.o_rlast
  //unused signals
  assert (core.io.o_r_id   === 0.U)
  assert (core.io.o_r_user === 0.U) 
}  */

  
}
