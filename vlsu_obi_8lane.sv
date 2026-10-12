// =============================================================================
// vlsu_obi_8lane.sv  –  8-Lane Vector LSU over OBI bus
// =============================================================================
// Instruction set supported (vlsu_pkg::vlsu_insn_e):
//   Loads  : VLE  (unit-stride), VLSE (strided), VLXE (indexed)
//   Stores : VSE  (unit-stride), VSSE (strided), VSXE (indexed)
//
// Lane-striped mapping : element i → lane (i % NrLanes)
// One outstanding OBI request at a time (matches cpu64_obi_receiver semantics).
//
// OBI bus wires (obi_*) are intended to be connected to the *core-side* ports
// of cpu64_obi_receiver.  The receiver's l1_* ports connect onward to L1 cache.
// =============================================================================

import vlsu_pkg::*;

module vlsu_obi_8lane #(
  parameter int unsigned NrLanes    = 8,
  parameter int unsigned ADDR_W     = 39,    // matches cpu64_obi_receiver CORE_ADDR_W
  parameter int unsigned ELEM_W     = 64,    // max element width in bits
  parameter int unsigned VLEN_ELEMS = 512    // max elements per vector op
)(
  input  logic clk_i,
  input  logic rst_ni,

  // -----------------------------------------------------------------------
  // Command interface  (from Global Sequencer / Dispatcher)
  // -----------------------------------------------------------------------
  input  logic                              cmd_valid_i,
  output logic                              cmd_ready_o,

  input  vlsu_insn_e                        cmd_insn_i,         // VLE/VLSE/VLXE/VSE/VSSE/VSXE
  input  logic [ADDR_W-1:0]                cmd_base_addr_i,
  input  logic [ADDR_W-1:0]                cmd_stride_bytes_i, // VLSE / VSSE only
  input  logic [$clog2(VLEN_ELEMS+1)-1:0]  cmd_vl_i,
  input  logic [$clog2(VLEN_ELEMS+1)-1:0]  cmd_vstart_i,
  input  logic [2:0]                        cmd_elem_bytes_i,   // 1/2/4/8
  input  logic                              cmd_vm_i,           // 1=no mask, 0=use mask
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

  // Stores: lane provides element data
  input  logic [NrLanes-1:0]               st_valid_i,
  output logic [NrLanes-1:0]               st_ready_o,
  input  logic [NrLanes-1:0][ELEM_W-1:0]  st_data_i,

  // Loads: LSU delivers element data to lanes
  output logic [NrLanes-1:0]               ld_valid_o,
  input  logic [NrLanes-1:0]               ld_ready_i,
  output logic [NrLanes-1:0][ELEM_W-1:0]  ld_data_o,

  // Indexed addressing: per-lane index vector (VLXE / VSXE).
  // Lane supplies one index element per LSU request cycle.
  // idx_valid_i[lane] must be high when LSU requests; LSU pulses idx_ready_o[lane].
  input  logic [NrLanes-1:0]               idx_valid_i,
  output logic [NrLanes-1:0]               idx_ready_o,
  input  logic [NrLanes-1:0][ELEM_W-1:0]  idx_data_i,   // byte offsets from base

  // -----------------------------------------------------------------------
  // OBI bus (connect to cpu64_obi_receiver core-side ports)
  //   obi_req_o   → req_i
  //   obi_we_o    → we_i
  //   obi_be_o    → be_i
  //   obi_addr_o  → addr_i
  //   obi_wdata_o → wdata_i
  //   obi_gnt_i   ← gnt_o
  //   obi_rvalid_i← rvalid_o
  //   obi_rdata_i ← rdata_o
  //   obi_err_i   — tie 0 or connect external error signal
  // -----------------------------------------------------------------------
  output logic                  obi_req_o,
  input  logic                  obi_gnt_i,
  output logic [ADDR_W-1:0]    obi_addr_o,
  output logic                  obi_we_o,
  output logic [ELEM_W-1:0]    obi_wdata_o,
  output logic [ELEM_W/8-1:0]  obi_be_o,

  input  logic                  obi_rvalid_i,
  input  logic [ELEM_W-1:0]    obi_rdata_i,
  input  logic                  obi_err_i
);

  // =========================================================================
  // Local parameters & typedefs
  // =========================================================================
  localparam int unsigned ELEM_B    = ELEM_W / 8;
  localparam int unsigned LOG_LANES = $clog2(NrLanes);
  localparam int unsigned LOG_VL    = $clog2(VLEN_ELEMS + 1);

  typedef enum logic [2:0] {
    S_IDLE,
    S_PREP,
    S_FETCH_IDX,    // NEW: wait for index element from lane (VLXE/VSXE)
    S_COLLECT_ST,
    S_ISSUE,
    S_WAIT_RSP,
    S_PUSH_LD,
    S_DONE,
    S_EXC
  } state_t;

  // =========================================================================
  // Helper functions
  // =========================================================================

  function automatic logic [ELEM_B-1:0] be_from_nbytes (input logic [2:0] n);
    logic [ELEM_B-1:0] be;
    int i;
    begin
      be = '0;
      for (i = 0; i < ELEM_B; i++)
        if (i < int'(n)) be[i] = 1'b1;
      return be;
    end
  endfunction

  function automatic logic misaligned (input logic [ADDR_W-1:0] addr, input logic [2:0] n);
    logic [2:0] mask;
    begin
      case (n)
        3'd1:    mask = 3'b000;
        3'd2:    mask = 3'b001;
        3'd4:    mask = 3'b011;
        3'd8:    mask = 3'b111;
        default: mask = 3'b111;
      endcase
      return |(addr[2:0] & mask);
    end
  endfunction

  // =========================================================================
  // Registers
  // =========================================================================
  state_t state_q, state_d;

  // Latched command fields
  vlsu_insn_e              insn_q;
  logic [ADDR_W-1:0]       base_q, stride_q;
  logic [LOG_VL-1:0]       vl_q, vstart_q;
  logic [2:0]              ebytes_q;   // element size in bytes
  logic                    vm_q;
  logic [NrLanes-1:0][ELEM_B-1:0] mask_q;

  // Derived from insn_q (combinational)
  logic is_store_w, is_strided_w, is_indexed_w;
  assign is_store_w   = insn_is_store  (insn_q);
  assign is_strided_w = insn_is_strided(insn_q);
  assign is_indexed_w = insn_is_indexed(insn_q);

  // Element / lane progress
  logic [LOG_VL-1:0]      elem_q, elem_d;
  logic [LOG_LANES-1:0]   lane_q, lane_d;

  // Index offset captured from lane (VLXE/VSXE)
  logic [ELEM_W-1:0]      idx_hold_q, idx_hold_d;
  logic                    idx_hold_valid_q, idx_hold_valid_d;

  // Store holding register
  logic [ELEM_W-1:0]      st_hold_q, st_hold_d;
  logic                    st_hold_valid_q, st_hold_valid_d;

  // Load holding register
  logic [ELEM_W-1:0]      ld_hold_q, ld_hold_d;
  logic                    ld_hold_valid_q, ld_hold_valid_d;
  logic [LOG_LANES-1:0]   ld_hold_lane_q, ld_hold_lane_d;

  // Post-load next-state (so S_PUSH_LD knows where to go)
  state_t                  after_push_q, after_push_d;

  // Exception latches
  logic                    exc_v_q,  exc_v_d;
  logic [ADDR_W-1:0]       exc_ba_q, exc_ba_d;
  logic [LOG_VL-1:0]       exc_vs_q, exc_vs_d;
  logic                    exc_ma_q, exc_ma_d;
  logic                    exc_be_q, exc_be_d;

  // =========================================================================
  // Address generation
  // =========================================================================
  logic [ADDR_W-1:0] cur_addr;

  always_comb begin
    if (is_indexed_w)
      cur_addr = base_q + ADDR_W'(idx_hold_q);   // base + per-element offset
    else if (is_strided_w)
      cur_addr = base_q + ADDR_W'(elem_q) * stride_q;
    else
      cur_addr = base_q + ADDR_W'(elem_q) * ADDR_W'(ebytes_q);
  end

  // Per-element byte-enable (masked if vm==0)
  logic [ELEM_B-1:0] elem_be;
  always_comb begin
    logic [ELEM_B-1:0] base_be;
    base_be = be_from_nbytes(ebytes_q);
    elem_be  = vm_q ? base_be : (base_be & mask_q[lane_q]);
  end

  // =========================================================================
  // Combinational FSM
  // =========================================================================
  always_comb begin
    // ---- defaults ----
    state_d          = state_q;
    elem_d           = elem_q;
    lane_d           = lane_q;

    idx_hold_d       = idx_hold_q;
    idx_hold_valid_d = idx_hold_valid_q;

    st_hold_d        = st_hold_q;
    st_hold_valid_d  = st_hold_valid_q;

    ld_hold_d        = ld_hold_q;
    ld_hold_valid_d  = ld_hold_valid_q;
    ld_hold_lane_d   = ld_hold_lane_q;
    after_push_d     = after_push_q;

    exc_v_d          = exc_v_q;
    exc_ba_d         = exc_ba_q;
    exc_vs_d         = exc_vs_q;
    exc_ma_d         = exc_ma_q;
    exc_be_d         = exc_be_q;

    // Outputs
    cmd_ready_o  = (state_q == S_IDLE);
    done_valid_o = 1'b0;

    ld_valid_o   = '0;
    ld_data_o    = '0;
    st_ready_o   = '0;
    idx_ready_o  = '0;

    obi_req_o    = 1'b0;
    obi_we_o     = 1'b0;
    obi_addr_o   = cur_addr;
    obi_wdata_o  = st_hold_q;
    obi_be_o     = elem_be;

    // Drive buffered load to its destination lane (back-pressurable)
    if (ld_hold_valid_q) begin
      ld_valid_o[ld_hold_lane_q] = 1'b1;
      ld_data_o [ld_hold_lane_q] = ld_hold_q;
      if (ld_ready_i[ld_hold_lane_q])
        ld_hold_valid_d = 1'b0;
    end

    // ------------------------------------------------------------------
    case (state_q)

      // ----------------------------------------------------------------
      S_IDLE: begin
        elem_d           = '0;
        lane_d           = '0;
        st_hold_valid_d  = 1'b0;
        ld_hold_valid_d  = 1'b0;
        idx_hold_valid_d = 1'b0;
        exc_v_d          = 1'b0;
        exc_ba_d         = '0;
        exc_vs_d         = '0;
        exc_ma_d         = 1'b0;
        exc_be_d         = 1'b0;

        if (cmd_valid_i)
          state_d = S_PREP;
      end

      // ----------------------------------------------------------------
      // S_PREP: set elem/lane to vstart; check initial alignment
      // ----------------------------------------------------------------
      S_PREP: begin
        elem_d = vstart_q;
        lane_d = LOG_LANES'(vstart_q % NrLanes);

        if (misaligned(cur_addr, ebytes_q)) begin
          exc_v_d  = 1'b1;
          exc_ba_d = cur_addr;
          exc_vs_d = vstart_q;
          exc_ma_d = 1'b1;
          state_d  = S_EXC;
        end else begin
          // Route to appropriate first state based on instruction type
          if (is_indexed_w)
            state_d = S_FETCH_IDX;
          else if (is_store_w)
            state_d = S_COLLECT_ST;
          else
            state_d = S_ISSUE;
        end
      end

      // ----------------------------------------------------------------
      // S_FETCH_IDX: consume one index element from the current lane.
      // For indexed stores we also need the store data, collected afterwards.
      // ----------------------------------------------------------------
      S_FETCH_IDX: begin
        idx_ready_o[lane_q] = 1'b1;

        if (idx_valid_i[lane_q] && !idx_hold_valid_q) begin
          idx_hold_d       = idx_data_i[lane_q];
          idx_hold_valid_d = 1'b1;
        end

        if (idx_hold_valid_d) begin
          state_d = is_store_w ? S_COLLECT_ST : S_ISSUE;
        end
      end

      // ----------------------------------------------------------------
      // S_COLLECT_ST: grab store data from the current lane
      // ----------------------------------------------------------------
      S_COLLECT_ST: begin
        if (!st_hold_valid_q) begin
          st_ready_o[lane_q] = 1'b1;

          if (st_valid_i[lane_q]) begin
            st_hold_d       = st_data_i[lane_q];
            st_hold_valid_d = 1'b1;
          end
        end

        if (st_hold_valid_d)
          state_d = S_ISSUE;
      end

      // ----------------------------------------------------------------
      // S_ISSUE: drive OBI request until granted
      // ----------------------------------------------------------------
      S_ISSUE: begin
        // Alignment check on every element (strict)
        if (misaligned(cur_addr, ebytes_q)) begin
          exc_v_d  = 1'b1;
          exc_ba_d = cur_addr;
          exc_vs_d = elem_q;
          exc_ma_d = 1'b1;
          state_d  = S_EXC;
        end else begin
          obi_req_o    = 1'b1;
          obi_we_o     = is_store_w;
          obi_addr_o   = cur_addr;
          obi_wdata_o  = st_hold_q;
          obi_be_o     = elem_be;

          if (obi_gnt_i)
            state_d = S_WAIT_RSP;
        end
      end

      // ----------------------------------------------------------------
      // S_WAIT_RSP: wait for OBI rvalid
      // ----------------------------------------------------------------
      S_WAIT_RSP: begin
        if (obi_rvalid_i) begin

          if (obi_err_i) begin
            exc_v_d  = 1'b1;
            exc_ba_d = cur_addr;
            exc_vs_d = elem_q;
            exc_be_d = 1'b1;
            state_d  = S_EXC;

          end else begin

            // On a successful load, buffer the result
            if (!is_store_w) begin
              ld_hold_d       = obi_rdata_i;
              ld_hold_valid_d = 1'b1;
              ld_hold_lane_d  = lane_q;
            end else begin
              st_hold_valid_d  = 1'b0;
            end

            // Clear the index hold so S_FETCH_IDX can accept the next one
            if (is_indexed_w)
              idx_hold_valid_d = 1'b0;

            // ----------------------------------------------------------
            // Advance element counter or finish
            // ----------------------------------------------------------
            if (elem_q + 1 >= (vstart_q + vl_q)) begin
              // Last element – go to done (possibly via S_PUSH_LD)
              if (!is_store_w) begin
                after_push_d = S_DONE;
                state_d      = S_PUSH_LD;
              end else begin
                state_d = S_DONE;
              end
            end else begin
              elem_d = elem_q + 1;
              lane_d = (lane_q == LOG_LANES'(NrLanes-1)) ? '0 : (lane_q + 1);

              if (!is_store_w) begin
                // Loads: push buffered data then issue next
                after_push_d = is_indexed_w ? S_FETCH_IDX : S_ISSUE;
                state_d      = S_PUSH_LD;
              end else begin
                // Stores: collect next element (and maybe fetch index first)
                state_d = is_indexed_w ? S_FETCH_IDX : S_COLLECT_ST;
              end
            end
          end
        end
      end

      // ----------------------------------------------------------------
      // S_PUSH_LD: hold until lane accepts the buffered load word
      // ----------------------------------------------------------------
      S_PUSH_LD: begin
        // Outputs already driven by ld_hold_valid_q block above.
        if (!ld_hold_valid_q) begin
          // Lane accepted; proceed to whatever comes next
          state_d = after_push_q;
        end
        // else stay here (backpressure)
      end

      // ----------------------------------------------------------------
      S_DONE: begin
        done_valid_o = 1'b1;
        if (done_ready_i)
          state_d = S_IDLE;
      end

      // ----------------------------------------------------------------
      S_EXC: begin
        // Report as done so the sequencer can handle the fault
        done_valid_o = 1'b1;
        if (done_ready_i)
          state_d = S_IDLE;
      end

      default: state_d = S_IDLE;

    endcase
  end

  // =========================================================================
  // Sequential
  // =========================================================================
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q          <= S_IDLE;
      elem_q           <= '0;
      lane_q           <= '0;
      insn_q           <= VLE;
      base_q           <= '0;
      stride_q         <= '0;
      vl_q             <= '0;
      vstart_q         <= '0;
      ebytes_q         <= 3'd8;
      vm_q             <= 1'b1;
      mask_q           <= '1;
      idx_hold_q       <= '0;
      idx_hold_valid_q <= 1'b0;
      st_hold_q        <= '0;
      st_hold_valid_q  <= 1'b0;
      ld_hold_q        <= '0;
      ld_hold_valid_q  <= 1'b0;
      ld_hold_lane_q   <= '0;
      after_push_q     <= S_IDLE;
      exc_v_q          <= 1'b0;
      exc_ba_q         <= '0;
      exc_vs_q         <= '0;
      exc_ma_q         <= 1'b0;
      exc_be_q         <= 1'b0;
    end else begin
      state_q          <= state_d;
      elem_q           <= elem_d;
      lane_q           <= lane_d;
      idx_hold_q       <= idx_hold_d;
      idx_hold_valid_q <= idx_hold_valid_d;
      st_hold_q        <= st_hold_d;
      st_hold_valid_q  <= st_hold_valid_d;
      ld_hold_q        <= ld_hold_d;
      ld_hold_valid_q  <= ld_hold_valid_d;
      ld_hold_lane_q   <= ld_hold_lane_d;
      after_push_q     <= after_push_d;
      exc_v_q          <= exc_v_d;
      exc_ba_q         <= exc_ba_d;
      exc_vs_q         <= exc_vs_d;
      exc_ma_q         <= exc_ma_d;
      exc_be_q         <= exc_be_d;

      // Accept command when IDLE
      if (cmd_valid_i && cmd_ready_o) begin
        insn_q    <= cmd_insn_i;
        base_q    <= cmd_base_addr_i;
        stride_q  <= cmd_stride_bytes_i;
        vl_q      <= cmd_vl_i;
        vstart_q  <= cmd_vstart_i;
        ebytes_q  <= cmd_elem_bytes_i;
        vm_q      <= cmd_vm_i;
        mask_q    <= cmd_mask_i;
      end
    end
  end

  // =========================================================================
  // Exception output assignments
  // =========================================================================
  assign exc_valid_o      = exc_v_q;
  assign exc_badaddr_o    = exc_ba_q;
  assign exc_vstart_o     = exc_vs_q;
  assign exc_misaligned_o = exc_ma_q;
  assign exc_bus_o        = exc_be_q;

endmodule
