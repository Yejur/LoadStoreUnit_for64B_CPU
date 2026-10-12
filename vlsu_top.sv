// =============================================================================
// vlsu_top.sv  –  Top-level wrapper
//
// Connects:
//   vlsu_obi_8lane  (Vector LSU)
//       ↓  OBI core-side
//   cpu64_obi_receiver  (OBI shim)
//       ↓  L1 CPU-side interface
//   L1 cache / fabric
//
// Parameters mirror both sub-modules; ADDR_W must equal CORE_ADDR_W (39-bit).
// =============================================================================

import vlsu_pkg::*;

module vlsu_top #(
  parameter int unsigned NrLanes    = 8,
  parameter int unsigned ADDR_W     = 39,
  parameter int unsigned ELEM_W     = 64,
  parameter int unsigned VLEN_ELEMS = 512
)(
  input  logic clk_i,
  input  logic rst_ni,

  // -----------------------------------------------------------------------
  // Command
  // -----------------------------------------------------------------------
  input  logic                              cmd_valid_i,
  output logic                              cmd_ready_o,
  input  vlsu_insn_e                        cmd_insn_i,
  input  logic [ADDR_W-1:0]               cmd_base_addr_i,
  input  logic [ADDR_W-1:0]               cmd_stride_bytes_i,
  input  logic [$clog2(VLEN_ELEMS+1)-1:0]  cmd_vl_i,
  input  logic [$clog2(VLEN_ELEMS+1)-1:0]  cmd_vstart_i,
  input  logic [2:0]                        cmd_elem_bytes_i,
  input  logic                              cmd_vm_i,
  input  logic [NrLanes-1:0][ELEM_W/8-1:0] cmd_mask_i,

  // -----------------------------------------------------------------------
  // Completion / exception
  // -----------------------------------------------------------------------
  output logic                              done_valid_o,
  input  logic                              done_ready_i,
  output logic                              exc_valid_o,
  output logic [ADDR_W-1:0]               exc_badaddr_o,
  output logic [$clog2(VLEN_ELEMS+1)-1:0]  exc_vstart_o,
  output logic                              exc_misaligned_o,
  output logic                              exc_bus_o,

  // -----------------------------------------------------------------------
  // Lane interfaces
  // -----------------------------------------------------------------------
  input  logic [NrLanes-1:0]               st_valid_i,
  output logic [NrLanes-1:0]               st_ready_o,
  input  logic [NrLanes-1:0][ELEM_W-1:0]  st_data_i,

  output logic [NrLanes-1:0]               ld_valid_o,
  input  logic [NrLanes-1:0]               ld_ready_i,
  output logic [NrLanes-1:0][ELEM_W-1:0]  ld_data_o,

  // Index vector (VLXE / VSXE)
  input  logic [NrLanes-1:0]               idx_valid_i,
  output logic [NrLanes-1:0]               idx_ready_o,
  input  logic [NrLanes-1:0][ELEM_W-1:0]  idx_data_i,

  // -----------------------------------------------------------------------
  // L1 cache interface (cpu64_obi_receiver l1_* side)
  // -----------------------------------------------------------------------
  input  logic             l1_gnt_i,
  input  logic             l1_rvalid_i,
  input  logic [63:0]      l1_rdata_i,

  output logic             l1_req_o,
  output logic             l1_we_o,
  output logic [7:0]       l1_be_o,
  output logic [63:0]      l1_addr_o,
  output logic [63:0]      l1_wdata_o
);

  // -----------------------------------------------------------------------
  // Internal OBI wires between LSU and OBI receiver
  // -----------------------------------------------------------------------
  logic                 obi_req,   obi_gnt;
  logic [ADDR_W-1:0]   obi_addr;
  logic                 obi_we;
  logic [ELEM_W-1:0]   obi_wdata;
  logic [ELEM_W/8-1:0] obi_be;
  logic                 obi_rvalid;
  logic [ELEM_W-1:0]   obi_rdata;

  // No external error path in cpu64_obi_receiver — tie low.
  // Extend here if you add an error signal to the receiver.
  logic obi_err;
  assign obi_err = 1'b0;

  // -----------------------------------------------------------------------
  // LSU instance
  // -----------------------------------------------------------------------
  vlsu_obi_8lane #(
    .NrLanes    (NrLanes),
    .ADDR_W     (ADDR_W),
    .ELEM_W     (ELEM_W),
    .VLEN_ELEMS (VLEN_ELEMS)
  ) u_lsu (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),

    .cmd_valid_i        (cmd_valid_i),
    .cmd_ready_o        (cmd_ready_o),
    .cmd_insn_i         (cmd_insn_i),
    .cmd_base_addr_i    (cmd_base_addr_i),
    .cmd_stride_bytes_i (cmd_stride_bytes_i),
    .cmd_vl_i           (cmd_vl_i),
    .cmd_vstart_i       (cmd_vstart_i),
    .cmd_elem_bytes_i   (cmd_elem_bytes_i),
    .cmd_vm_i           (cmd_vm_i),
    .cmd_mask_i         (cmd_mask_i),

    .done_valid_o       (done_valid_o),
    .done_ready_i       (done_ready_i),
    .exc_valid_o        (exc_valid_o),
    .exc_badaddr_o      (exc_badaddr_o),
    .exc_vstart_o       (exc_vstart_o),
    .exc_misaligned_o   (exc_misaligned_o),
    .exc_bus_o          (exc_bus_o),

    .st_valid_i         (st_valid_i),
    .st_ready_o         (st_ready_o),
    .st_data_i          (st_data_i),

    .ld_valid_o         (ld_valid_o),
    .ld_ready_i         (ld_ready_i),
    .ld_data_o          (ld_data_o),

    .idx_valid_i        (idx_valid_i),
    .idx_ready_o        (idx_ready_o),
    .idx_data_i         (idx_data_i),

    // OBI core-side
    .obi_req_o          (obi_req),
    .obi_gnt_i          (obi_gnt),
    .obi_addr_o         (obi_addr),
    .obi_we_o           (obi_we),
    .obi_wdata_o        (obi_wdata),
    .obi_be_o           (obi_be),
    .obi_rvalid_i       (obi_rvalid),
    .obi_rdata_i        (obi_rdata),
    .obi_err_i          (obi_err)
  );

  // -----------------------------------------------------------------------
  // OBI receiver instance
  // -----------------------------------------------------------------------
  cpu64_obi_receiver #(
    .CORE_ADDR_W (ADDR_W),
    .DATA_W      (ELEM_W)
  ) u_obi_rx (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    // Core-side (from LSU)
    .req_i      (obi_req),
    .we_i       (obi_we),
    .be_i       (obi_be),
    .addr_i     (obi_addr),
    .wdata_i    (obi_wdata),
    .gnt_o      (obi_gnt),
    .rvalid_o   (obi_rvalid),
    .rdata_o    (obi_rdata),

    // L1 cache side
    .l1_req_o   (l1_req_o),
    .l1_we_o    (l1_we_o),
    .l1_be_o    (l1_be_o),
    .l1_addr_o  (l1_addr_o),
    .l1_wdata_o (l1_wdata_o),
    .l1_gnt_i   (l1_gnt_i),
    .l1_rvalid_i(l1_rvalid_i),
    .l1_rdata_i (l1_rdata_i)
  );

endmodule
