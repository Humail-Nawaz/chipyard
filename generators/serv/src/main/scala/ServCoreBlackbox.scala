//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Serv Core Blackbox
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
package serv

import sys.process._

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, StringParam, RawParam}

import scala.collection.mutable.{ListBuffer}

class ServCoreBlackbox(
	memfile_b: String,
	memsize_b: Int,
	sim_b: Int,
	reset_strategy_b: String,
	with_csr_b: Int,
    	aw_b: Int,
	uw_b  : Int,
	iw_b  : Int	
)

extends BlackBox(
   Map(
	"MEMFILE_B"        -> StringParam(memfile_b),
	"MEMSIZE_B"        -> IntParam(memsize_b),
	"SIM_B"            -> IntParam(sim_b),
	"RESET_STRATEGY_B" -> StringParam(reset_strategy_b),
	"WITH_CSR_B"	   -> IntParam(with_csr_b),
	"AW_B"             -> IntParam(aw_b),
	"USER_WIDTH"       -> IntParam(uw_b),
	"ID_WIDTH"	   -> IntParam(iw_b)
   ) )


  with HasBlackBoxPath
  {
  val io = IO ( new Bundle {
	  
  		val  clk	 = Input(Clock())
  		val  rst	 = Input(Bool())
  		val  i_timer_irq = Input(Bool())
	  
  		/*val  i_awaddr    = Input(UInt(aw_b.W))
		val  i_awvalid   = Input(Bool())
		val  o_awready   = Output(Bool())
	        val  i_aw_id	 = Input(UInt(iw_b.W))
	        val  i_aw_len    = Input(UInt(8.W))
	        val  i_aw_size   = Input(UInt(4.W))
	        val  i_aw_burst  = Input(UInt(2.W))
	        val  i_aw_lock   = Input(Bool())
	        val  i_aw_cache  = Input(UInt(4.W))
	        val  i_aw_prot   = Input(UInt(3.W))
	        val  i_aw_qos    = Input(UInt(4.W))
	        val  i_aw_region = Input(UInt(4.W))
	        val  i_aw_atop   = Input(UInt(6.W))
	        val  i_aw_user   = Input(UInt(uw_b.W))
	  
		val  i_araddr    = Input(UInt(aw_b.W))
		val  i_arvalid   = Input(Bool())
		val  o_arready   = Output(Bool())
                val  i_ar_id     = Input(UInt(iw_b.W))
	        val  i_ar_len    = Input(UInt(8.W))
	        val  i_ar_size   = Input(UInt(4.W))
	        val  i_ar_burst  = Input(UInt(2.W))
	        val  i_ar_lock   = Input(Bool())
	        val  i_ar_cache  = Input(UInt(4.W))
	        val  i_ar_prot   = Input(UInt(3.W))
	        val  i_ar_qos    = Input(UInt(4.W))
	        val  i_ar_region = Input(UInt(4.W))
	        val  i_ar_user   = Input(UInt(uw_b.W))
	  
		val  i_wdata     = Input(UInt(32.W))
		val  i_wstrb     = Input(UInt(4.W))
		val  i_wvalid    = Input(Bool())
		val  o_wready    = Output(Bool())
                val  i_w_last    = Input(Bool())
	        val  i_w_user    = Input(UInt(uw_b.W))
	  
		val  i_bready    = Input(Bool())
	        val  o_bresp     = Output(UInt(2.W))
	        val  o_bvalid    = Output(Bool())
                val  o_b_id      = Output(UInt(iw_b.W))
	        val  o_b_user    = Output(UInt(uw_b.W))
	  
	        val  i_rready    = Input(Bool())
	        val  o_rdata     = Output(UInt(32.W))
	        val  o_rresp     = Output(UInt(2.W))
	  	val  o_rlast	 = Output(Bool())
	  	val  o_rvalid    = Output(Bool())
                val  o_r_id      = Output(UInt(iw_b.W))
	        val  o_r_user    = Output(UInt(uw_b.W))*/
	  
	  	val  i_awmready  = Input(Bool())
	        val  o_awmaddr   = Output(UInt(12.W))
	        val  o_awmvalid  = Output(Bool())
                val  o_awm_id    = Output(UInt(iw_b.W)) 
                val  o_awm_len   = Output(UInt(8.W))
                val  o_awm_size  = Output(UInt(3.W))
                val  o_awm_burst = Output(UInt(2.W))
                val  o_awm_lock  = Output(Bool())
                val  o_awm_cache = Output(UInt(4.W))
                val  o_awm_prot  = Output(UInt(3.W))
                val  o_awm_qos   = Output(UInt(4.W))
                val  o_awm_region= Output(UInt(4.W))
                val  o_awm_atop  = Output(UInt(6.W))
                val  o_awm_user  = Output(UInt(uw_b.W)) 
	  
	        val  i_armready  = Input(Bool())
	        val  o_armaddr   = Output(UInt(12.W))
	  	val  o_armvalid  = Output(Bool())
                val  o_arm_id    = Output(UInt(iw_b.W)) 
                val  o_arm_len   = Output(UInt(8.W))
                val  o_arm_size  = Output(UInt(3.W))
                val  o_arm_burst = Output(UInt(2.W))
                val  o_arm_lock  = Output(Bool())
                val  o_arm_cache = Output(UInt(4.W))
                val  o_arm_prot  = Output(UInt(3.W))
                val  o_arm_qos   = Output(UInt(4.W))
                val  o_arm_region= Output(UInt(4.W))
                val  o_arm_user  = Output(UInt(uw_b.W))
	  
	  	val  i_wmready   = Input(Bool())
	  	val  o_wmdata	 = Output(UInt(32.W))
	  	val  o_wmstrb    = Output(UInt(4.W))
	  	val  o_wmvalid   = Output(Bool())
                val  o_wm_last   = Output(Bool())
                val  o_wm_user   = Output(UInt(uw_b.W))
        
	  
	  	val  i_bmresp	 = Input(UInt(2.W))
	  	val  i_bmvalid   = Input(Bool())
	  	val  o_bmready   = Output(Bool())
                val  i_bm_id     = Input(UInt(iw_b.W))
                val  i_bm_user   = Input(UInt(uw_b.W))
	  
	  	val  i_rmdata    = Input(UInt(32.W))
	  	val  i_rmresp    = Input(UInt(2.W))
	  	val  i_rmlast	 = Input(Bool())
	  	val  i_rmvalid   = Input(Bool())
	  	val  o_rmready   = Output(Bool())
	        val  i_rm_id     = Input(UInt(iw_b.W))
                val  i_rm_user   = Input(UInt(uw_b.W))
}  )

	  
    val chipyardDir = System.getProperty("user.dir")
    val servVsrcDir = s"$chipyardDir/generators/serv/src/main/resources/vsrc"

    val proc = s"make -C $servVsrcDir default"
    require (proc.! == 0, "Failed to run preprocessing step")

    // generated from preprocessing step
    addPath(s"$servVsrcDir/ServCoreBlackbox.preprocessed.v")
    
  }
