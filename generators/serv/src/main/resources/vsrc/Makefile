#------------------------------------------------------------------------------
# Makefile to generate a preprocessed Verilog file for the SERV core
#------------------------------------------------------------------------------

BASE_DIR := $(abspath ../../../../../..)
VSRC_DIR := $(abspath .)
SERV_DIR := $(VSRC_DIR)/serv

# Output preprocessed Verilog
PREPROC_VERILOG = ServCoreBlackbox.preprocessed.v

.PHONY: default $(PREPROC_VERILOG) clean
default: $(PREPROC_VERILOG)

# SERV RTL source files
SERV_VSRCS = \
	$(SERV_DIR)/rtl/serv_alu.v \
	$(SERV_DIR)/rtl/serv_bufreg.v \
	$(SERV_DIR)/rtl/serv_bufreg2.v \
	$(SERV_DIR)/rtl/serv_ctrl.v \
	$(SERV_DIR)/rtl/serv_decode.v \
	$(SERV_DIR)/rtl/serv_immdec.v \
	$(SERV_DIR)/rtl/serv_mem_if.v \
	$(SERV_DIR)/rtl/serv_rf_if.v \
	$(SERV_DIR)/rtl/serv_state.v \
	$(SERV_DIR)/rtl/serv_top.v \
	$(SERV_DIR)/rtl/serv_rf_ram_if.v \
	$(SERV_DIR)/rtl/serving_ram.v \
	$(SERV_DIR)/rtl/serving.v \
	$(SERV_DIR)/rtl/servile_rf_mem_if.v \
        $(SERV_DIR)/rtl/servile.v\
        $(SERV_DIR)/rtl/servile_arbiter.v\
        $(SERV_DIR)/rtl/servile_mux.v\
        $(SERV_DIR)/rtl/ServCore.v\
        $(SERV_DIR)/rtl/complete_bridge.v\
        $(SERV_DIR)/rtl/serv_aligner.v\
        $(SERV_DIR)/rtl/serv_compdec.v\
        $(SERV_DIR)/rtl/serv_csr.v\
        $(SERV_DIR)/rtl/serv_debug.v\
        $(SERV_DIR)/rtl/serv_synth_wrapper.v\
        $(SERV_DIR)/rtl/serv_rf_top.v

# Top-level wrapper file (matches Chisel blackbox declaration)
SERV_WRAPPER = $(VSRC_DIR)/ServCoreBlackbox.v

ALL_VSRCS = $(SERV_VSRCS) $(SERV_WRAPPER)

# Chipyard includes helper script
PREPROC_SCRIPT = $(BASE_DIR)/scripts/insert-includes.py

# Include directories (empty unless you use third-party IP)
lookup_dirs = $(shell find -L $(serv_dir) -name target -prune -o -type d -print 2> /dev/null | grep '.*/\($(1)\)$$')
INC_DIR_NAMES ?= rtl include formal vendor
INC_DIRS ?= $(foreach dir_name,$(INC_DIR_NAMES),$(call lookup_dirs,$(dir_name)))

# Defines for Verilator preprocessing
EXTRA_PREPROC_DEFINES ?=
PREPROC_DEFINES ?= \
	VERILATOR \
	$(EXTRA_PREPROC_DEFINES)

$(PREPROC_VERILOG): $(ALL_VSRCS)
	mkdir -p $(dir $@)
	# Create temp define/undef files
	$(foreach def,$(PREPROC_DEFINES),echo "\`define $(def)" >> def.v;)
	$(foreach def,$(PREPROC_DEFINES),echo "\`undef $(def)" >> undef.v;)
	# Combine files into single .sv
	cat def.v $(ALL_VSRCS) undef.v > combined.v
	# Insert `include statements
	$(PREPROC_SCRIPT) combined.v $@ $(INC_DIRS)
	# Optional: remove unsupported modules if needed for Verilator
	# sed -i '/.*unsupported_module/,/;/c\    // removed for verilator' $@
	# Cleanup
	rm -f combined.v def.v undef.v

clean:
	rm -f $(PREPROC_VERILOG)
