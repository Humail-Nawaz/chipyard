#------------------------------------------------------------------------------
# Makefile to generate a preprocessed Verilog file for the SERV core
#------------------------------------------------------------------------------

base_dir = $(abspath ../../../../../..)
vsrc_dir= $(abspath .)
serv_dir = $(vsrc_dir)/serv

# Output preprocessed Verilog
PREPROC_VERILOG = ServCoreBlackbox.preprocessed.v

.PHONY: default $(PREPROC_VERILOG) clean
default: $(PREPROC_VERILOG)

# SERV RTL source files
SERV_VSRCS = \
	$(serv_dir)/rtl/serv_alu.v \
	$(serv_dir)/rtl/serv_aligner.v \
	$(serv_dir)/rtl/serv_compdec.v \
	$(serv_dir)/rtl/serv_debug.v \
	$(serv_dir)/rtl/serv_csr.v \
	$(serv_dir)/rtl/serv_bufreg.v \
	$(serv_dir)/rtl/serv_bufreg2.v \
	$(serv_dir)/rtl/serv_ctrl.v \
	$(serv_dir)/rtl/serv_decode.v \
	$(serv_dir)/rtl/serv_immdec.v \
	$(serv_dir)/rtl/serv_rf_ram.v \
	$(serv_dir)/rtl/serv_mem_if.v \
	$(serv_dir)/rtl/serv_rf_if.v \
	$(serv_dir)/rtl/serv_state.v \
	$(serv_dir)/rtl/serv_top.v \
	$(serv_dir)/rtl/serv_rf_top.v \
	$(serv_dir)/rtl/serv_rf_ram_if.v \
	$(serv_dir)/rtl/serv_synth_wrapper.v \
	$(serv_dir)/rtl/serving_ram.v \
	$(serv_dir)/rtl/serving.v \
	$(serv_dir)/rtl/servile_rf_mem_if.v \
	$(serv_dir)/rtl/servile.v \
	$(serv_dir)/rtl/servile_mux.v \
	$(serv_dir)/rtl/servile_arbiter.v \
	$(serv_dir)/rtl/complete_bridge.v \
	$(serv_dir)/rtl/ServCore.v

  
# Top-level wrapper file (matches Chisel blackbox declaration)
SERV_WRAPPER = $(vsrc_dir)/ServCoreBlackbox.v

ALL_VSRCS = $(SERV_VSRCS) $(SERV_WRAPPER)

# Chipyard includes helper script
PREPROC_SCRIPT = $(base_dir)/scripts/insert-includes.py

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
