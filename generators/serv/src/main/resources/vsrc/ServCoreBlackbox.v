module ServCoreBlackbox 
#(
    parameter MEMFILE_B      = "",           // Hex file to be loaded into core RAM.
    parameter MEMSIZE_B      = 8192,
    parameter SIM_B          = 1'b0,
    parameter RESET_STRATEGY_B = "MINI",
    parameter WITH_CSR_B     = 1,
    parameter AW_B           = 13,
    parameter USER_WIDTH     = 0,
    parameter ID_WIDTH       = 0
)
(
    // Core and timer
    input   clk,
    input   rst,
    input   i_timer_irq,

    // AXI address write channel
    input   i_awmready,
    output  [AW_B-1:0] o_awmaddr,
    output  o_awmvalid,
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
    input   i_wmready,
    output  [31:0] o_wmdata,
    output  [3:0]  o_wmstrb,
    output  o_wmvalid,
    output  o_wm_last,
    output  [USER_WIDTH-1:0] o_wm_user,

    // AXI response channel
    input   [1:0] i_bmresp,
    input   i_bmvalid,
    output  o_bmready,
    input   [ID_WIDTH-1:0] i_bm_id,
    input   [USER_WIDTH-1:0] i_bm_user,

    // AXI read channel
    input   [31:0] i_rmdata,
    input   [1:0]  i_rmresp,
    input   i_rmlast,
    input   i_rmvalid,
    output  o_rmready,
    input   [ID_WIDTH-1:0] i_rm_id,
    input   [USER_WIDTH-1:0] i_rm_user
);

// Instantiate serving bridge SoC
ServCore #(
    .memfile      (MEMFILE_B),
    .memsize      (MEMSIZE_B),
    .sim          (SIM_B),
    .RESET_STRATEGY(RESET_STRATEGY_B),
    .WITH_CSR     (WITH_CSR_B),
    .AW           (AW_B),
    .ID_WIDTH     (ID_WIDTH),
    .USER_WIDTH   (USER_WIDTH)
) i_serving_bridge_top (
    .clk          (clk),
    .rst          (rst),
    .i_timer_irq  (i_timer_irq),

    .i_awmready   (i_awmready),
    .o_awmaddr    (o_awmaddr),
    .o_awmvalid   (o_awmvalid),
    .o_awm_id     (o_awm_id),
    .o_awm_len    (o_awm_len),
    .o_awm_size   (o_awm_size),
    .o_awm_burst  (o_awm_burst),
    .o_awm_lock   (o_awm_lock),
    .o_awm_cache  (o_awm_cache),
    .o_awm_prot   (o_awm_prot),
    .o_awm_qos    (o_awm_qos),
    .o_awm_region (o_awm_region),
    .o_awm_atop   (o_awm_atop),
    .o_awm_user   (o_awm_user),

    .i_armready   (i_armready),
    .o_armaddr    (o_armaddr),
    .o_armvalid   (o_armvalid),
    .o_arm_id     (o_arm_id),
    .o_arm_len    (o_arm_len),
    .o_arm_size   (o_arm_size),
    .o_arm_burst  (o_arm_burst),
    .o_arm_lock   (o_arm_lock),
    .o_arm_cache  (o_arm_cache),
    .o_arm_prot   (o_arm_prot),
    .o_arm_qos    (o_arm_qos),
    .o_arm_region (o_arm_region),
    .o_arm_user   (o_arm_user),

    .i_wmready    (i_wmready),
    .o_wmdata     (o_wmdata),
    .o_wmstrb     (o_wmstrb),
    .o_wmvalid    (o_wmvalid),
    .o_wm_last    (o_wm_last),
    .o_wm_user    (o_wm_user),

    .i_bmresp     (i_bmresp),
    .i_bmvalid    (i_bmvalid),
    .o_bmready    (o_bmready),
    .i_bm_id      (i_bm_id),
    .i_bm_user    (i_bm_user),

    .i_rmdata     (i_rmdata),
    .i_rmresp     (i_rmresp),
    .i_rmlast     (i_rmlast),
    .i_rmvalid    (i_rmvalid),
    .o_rmready    (o_rmready),
    .i_rm_id      (i_rm_id),
    .i_rm_user    (i_rm_user)
);

endmodule

