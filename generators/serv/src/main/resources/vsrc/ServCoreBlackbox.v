
module ServCoreBlackbox 
#(
   parameter MEMFILE_B = "",           // Hex file to be loaded into core RAM.
   parameter MEMSIZE_B = 8192,
   parameter SIM_B = 1'b0,
   parameter RESET_STRATEGY_B = "MINI",
   parameter WITH_CSR_B = 1,
   parameter AW_B       = 13,
   parameter USER_WIDTH = 0,
   parameter ID_WIDTH   = 0
)

(
    // CORE TOP
    input   clk,
    input   rst,
    input   i_timer_irq,

    // AXI2WB -- AXI SIGNALS FROM EXTERNAL(BUS/PERIPHERAL/ADAPTER) TO BRIDGE

    // AXI address write channel
    input   [AW_B-1:0] i_awaddr,
    input   i_awvalid,
    output  o_awready,
     //unused signals
    input  [ID_WIDTH-1:0] i_aw_id,
    input  [7:0] i_aw_len,
    input  [3:0] i_aw_size,
    input  [1:0] i_aw_burst,
    input  i_aw_lock,
    input  [3:0] i_aw_cache,
    input  [2:0] i_aw_prot,
    input  [3:0] i_aw_qos,
    input  [3:0] i_aw_region,
    input  [5:0] i_aw_atop,
    input  [USER_WIDTH-1:0] i_aw_user,

    // AXI address read channel 
    input   [AW_B-1:0] i_araddr,
    input   i_arvalid,
    output  o_arready,
    //unused signals
    input [ID_WIDTH-1:0] i_ar_id,
    input  [7:0] i_ar_len,
    input  [2:0] i_ar_size,
    input  [1:0] i_ar_burst,
    input  i_ar_lock,
    input  [3:0]i_ar_cache,
    input  [2:0]i_ar_prot,
    input  [3:0]i_ar_qos,
    input  [3:0]i_ar_region,
    input  [USER_WIDTH-1:0] i_ar_user,
   
    // AXI write channel
    input   [31:0] i_wdata,
    input   [3:0] i_wstrb,
    input   i_wvalid,
    output  o_wready,
   //unused signals
    input   i_w_last,
    input  [USER_WIDTH-1:0] i_w_user,

    // AXI response channel
    input   i_bready,
    output  [1:0] o_bresp,
    output  o_bvalid,
   //unused signals 
   output  [ID_WIDTH-1:0] o_b_id,
   output  [USER_WIDTH-1:0] o_b_user,
    
    // AXI read channel
    input   i_rready,
    output  [31:0] o_rdata,
    output  [1:0] o_rresp,
    output  o_rlast,
    output  o_rvalid,
    //unused signals
    output  [ID_WIDTH-1:0] o_r_id,
    output  [USER_WIDTH-1:0] o_r_user,
    // ---------------------------------------------------------------- //

    // AXI2WB AXI SIGNALS FROM BRIDGE TO EXTERNAL(PERIPHERAL/ADAPTER/BUS)


    // AXI address write channel
    input   i_awmready,
    output  [AW_B-1:0] o_awmaddr,
    output  o_awmvalid,
    //unused signals
    output  [ID_WIDTH-1:0] o_awm_id,
    output  [7:0] o_awm_len,
    output  [2:0] o_awm_size,
    output  [1:0] o_awm_burst,
    output  o_awm_lock,
    output  [3:0] o_awm_cache,
    output  [2:0] o_awm_prot,
    output  [3:0] o_awm_qos,
    output  [3:0] o_awm_region,
    output  [5:0] o_awm_atop,
    output  [USER_WIDTH-1:0] o_awm_user,

    // AXI address read channel
    input   i_armready,
    output  [AW_B-1:0] o_armaddr,
    output  o_armvalid,
    //unused signals
    output  [ID_WIDTH-1:0] o_arm_id,
    output  [7:0] o_arm_len,
    output  [2:0] o_arm_size,
    output  [1:0] o_arm_burst,
    output  o_arm_lock,
    output  [3:0] o_arm_cache,
    output  [2:0] o_arm_prot,
    output  [3:0] o_arm_qos,
    output  [3:0] o_arm_region,
    output  [USER_WIDTH-1:0] o_arm_user,

    // AXI write channel
    input  i_wmready,
    output [31:0] o_wmdata,
    output [3:0] o_wmstrb,
    output o_wmvalid,
    //unused signals
    output  o_wm_last,
    output  [USER_WIDTH-1:0] o_wm_user,

    // AXI response channel
    input  [1:0] i_bmresp,
    input  i_bmvalid,
    output o_bmready,
    //unused signals 
    input  [ID_WIDTH-1:0] i_bm_id,
    input  [USER_WIDTH-1:0] i_bm_user,
    
    //AXI read channel
    input   [31:0] i_rmdata,
    input   [1:0] i_rmresp,
    input   i_rmlast,
    input   i_rmvalid,
    output  o_rmready,
    //unused signals
    input wire [ID_WIDTH-1:0] i_rm_id,
    input wire [USER_WIDTH-1:0] i_rm_user

);


serving_bridge_top  // Serving (SoClet containing SERV and Servile Wrapper) plus the Bridge for conversion from Wishbone to AXI and vice versa when needed.
#(
        .memfile(MEMFILE_B),
        .memsize(MEMSIZE_B),
        .sim(SIM_B),
        .RESET_STRATEGY(RESET_STRATEGY_B),
        .WITH_CSR(WITH_CSR_B),
        .AW(AW_B),
        .ID_WIDTH(ID_WIDTH),
        .USER_WIDTH(USER_WIDTH)
)

i_serving_bridge_top (
   
    .clk(clk),
    .rst(rst),
    .i_timer_irq(i_timer_irq),

   // AXI SIGNALS FROM TILELINK---->BRIDGE---->SERVING
    .i_awaddr(i_awaddr),
    .i_awvalid(i_awvalid),
    .o_awready(o_awready),
    .i_aw_id(i_aw_id),
    .i_aw_len(i_aw_len),
    .i_aw_size(i_aw_size),
    .i_aw_burst(i_aw_burst),
    .i_aw_lock(i_aw_lock),
    .i_aw_cache(i_aw_cache),
    .i_aw_prot(i_aw_prot),
    .i_aw_qos(i_aw_qos),
    .i_aw_region(i_aw_region),
    .i_aw_atop(i_aw_atop),
    .i_aw_user(i_aw_user),

    .i_araddr(i_araddr),
    .i_arvalid(i_arvalid),
    .o_arready(o_arready),
    .i_ar_id(i_ar_id),
    .i_ar_len(i_ar_len),
    .i_ar_size(i_ar_size),
    .i_ar_burst(i_ar_burst),
    .i_ar_lock(i_ar_lock),
    .i_ar_cache(i_ar_cache),
    .i_ar_prot(i_ar_prot),
    .i_ar_qos(i_ar_qos),
    .i_ar_region(i_ar_region),
    .i_ar_user(i_ar_user),

    .i_wdata(i_wdata),
    .i_wstrb(i_wstrb),
    .i_wvalid(i_wvalid),
    .o_wready(o_wready),
    .i_w_last(i_w_last),
    .i_w_user(i_w_user),

    .o_bresp(o_bresp),
    .o_bvalid(o_bvalid),
    .i_bready(i_bready),
    .o_b_id(o_b_id),
    .o_b_user(o_b_user),

     .o_rdata(o_rdata),
    .o_rresp(o_rresp),
    .o_rlast(o_rlast),
    .o_rvalid(o_rvalid),
    .i_rready(i_rready),
    .o_r_id(o_r_id),
    .o_r_user(o_r_user),
   
// AXI SIGNALS FROM SERVING--->BRIDGE---->TILELINK
    .o_awmaddr(o_awmaddr),
    .o_awmvalid(o_awmvalid),
    .i_awmready(i_awmready),
    .o_awm_id(o_awm_id),
    .o_awm_len(o_awm_len),
    .o_awm_size(o_awm_size),
    .o_awm_burst(o_awm_burst),
    .o_awm_lock(o_awm_lock),
    .o_awm_cache(o_awm_cache),
    .o_awm_prot(o_awm_prot),
    .o_awm_qos(o_awm_qos),
    .o_awm_region(o_awm_region),
    .o_awm_atop(o_awm_atop),
    .o_awm_user(o_awm_user),
   
    .o_armaddr(o_armaddr),
    .o_armvalid(o_armvalid),
    .i_armready(i_armready),
    .o_arm_id(o_arm_id),
    .o_arm_len(o_arm_len),
    .o_arm_size(o_arm_size),
    .o_arm_burst(o_arm_burst),
    .o_arm_lock(o_arm_lock),
    .o_arm_cache(o_arm_cache),
    .o_arm_prot(o_arm_prot),
    .o_arm_qos(o_arm_qos),
    .o_arm_region(o_arm_region),
    .o_arm_user(o_arm_user),

    .o_wmdata(o_wmdata),
    .o_wmstrb(o_wmstrb),
    .o_wmvalid(o_wmvalid),
    .i_wmready(i_wmready),
    .o_wm_last(o_wm_last),
    .o_wm_user(o_wm_user),
   
    .i_bmresp(i_bmresp),
    .i_bmvalid(i_bmvalid),
    .o_bmready(o_bmready),
    .i_bm_id(i_bm_id),
    .i_bm_user(i_bm_user),

    .i_rmdata(i_rmdata),
    .i_rmresp(i_rmresp),
    .i_rmlast(i_rmlast),
    .i_rmvalid(i_rmvalid),
    .o_rmready(o_rmready),
    .i_rm_id(i_rm_id),
    .i_rm_user(i_rm_user)
    );

endmodule
