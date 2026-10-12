`timescale 1ns/1ps
// =============================================================================
// tb_vlsu.sv  —  Self-checking testbench for vlsu_top
//
// Tests covered:
//  TC1  : VLE   unit-stride load,  4 elements, 64-bit
//  TC2  : VSE   unit-stride store, 4 elements, 64-bit
//  TC3  : VLSE  strided load,      4 elements, stride=16 bytes
//  TC4  : VSSE  strided store,     4 elements, stride=16 bytes
//  TC5  : VLXE  indexed load,      4 elements
//  TC6  : VSXE  indexed store,     4 elements
//  TC7  : VLE   8-bit elements     (elem_bytes=1)
//  TC8  : VLE   with mask          (vm=0, some lanes byte-masked)
//  TC9  : VLE   vstart != 0        (restart from element 2)
//  TC10 : Misalignment exception   (odd base + 2-byte elem)
//  TC11 : Bus error exception
//  TC12 : VSE   back-pressure      (lane stalls 3 cycles before data)
// =============================================================================

module tb;
  import vlsu_pkg::*;

  // =========================================================================
  // Parameters
  // =========================================================================
  localparam int NrLanes    = 8;
  localparam int ADDR_W     = 39;
  localparam int ELEM_W     = 64;
  localparam int VLEN_ELEMS = 512;
  localparam int LOG_VL     = $clog2(VLEN_ELEMS+1);  // 10

  // =========================================================================
  // Clock & reset
  // =========================================================================
  logic clk = 0;
  logic rst_n;
  always #5 clk = ~clk;   // 100 MHz

  // =========================================================================
  // DUT ports
  // =========================================================================
  logic                             cmd_valid, cmd_ready;
  vlsu_insn_e                       cmd_insn;
  logic [ADDR_W-1:0]                cmd_base;
  logic [ADDR_W-1:0]                cmd_stride;
  logic [LOG_VL-1:0]                cmd_vl;
  logic [LOG_VL-1:0]                cmd_vstart;
  logic [3:0]                       cmd_ebytes;
  logic                             cmd_vm;
  logic [NrLanes-1:0][7:0]          cmd_mask;

  logic                             done_valid, done_ready;
  logic                             exc_valid;
  logic [ADDR_W-1:0]                exc_badaddr;
  logic [LOG_VL-1:0]                exc_vstart_out;
  logic                             exc_ma, exc_bus;

  logic [NrLanes-1:0]               st_valid, st_ready;
  logic [NrLanes-1:0][ELEM_W-1:0]  st_data;
  logic [NrLanes-1:0]               ld_valid, ld_ready;
  logic [NrLanes-1:0][ELEM_W-1:0]  ld_data;
  logic [NrLanes-1:0]               idx_valid, idx_ready;
  logic [NrLanes-1:0][ELEM_W-1:0]  idx_data;

  // L1 ports
  logic        l1_gnt_force;
  logic        l1_err_next;
  logic        l1_gnt, l1_rvalid;
  logic [63:0] l1_rdata;
  logic        l1_req, l1_we;
  logic [7:0]  l1_be;
  logic [63:0] l1_addr, l1_wdata;

  // =========================================================================
  // Test scoreboard
  // =========================================================================
  int tests_run  = 0;
  int tests_pass = 0;
  int tests_fail = 0;

  task automatic pass_t(input string name);
    $display("  [PASS] %s", name);
    tests_pass++; tests_run++;
  endtask

  task automatic fail_t(input string name, input string reason);
    $display("  [FAIL] %s  —  %s", name, reason);
    tests_fail++; tests_run++;
  endtask

  `define CHECK(label, got, exp) \
    if ((got) === (exp)) pass_t(label); \
    else fail_t(label, $sformatf("got=0x%0h  expected=0x%0h", (got), (exp)));

  // =========================================================================
  // L1 Memory Model  (4 KB, initialised with addr ^ 0xA5)
  // 2-cycle read latency.  Writes take effect immediately.
  // =========================================================================
  logic [7:0]  mem [0:4095];
  logic [1:0]  rv_pipe;
  logic [63:0] rd_pipe [0:1];
  logic        err_pipe[0:1];

  initial begin
    for (int i = 0; i < 4096; i++) mem[i] = i[7:0] ^ 8'hA5;
  end

  // Write
  always_ff @(posedge clk) begin
    if (l1_req && l1_we && l1_gnt)
      for (int b = 0; b < 8; b++)
        if (l1_be[b]) mem[(l1_addr[11:0] + b) % 4096] <= l1_wdata[b*8 +: 8];
  end

  // Read pipeline
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rv_pipe    <= 2'b00;
      rd_pipe[0] <= '0; rd_pipe[1] <= '0;
      err_pipe[0]<= 0;  err_pipe[1]<= 0;
    end else begin
      rv_pipe[0] <= l1_req && l1_gnt && !l1_we;
      if (l1_req && l1_gnt && !l1_we) begin
        for (int b = 0; b < 8; b++)
          rd_pipe[0][b*8 +: 8] <= mem[(l1_addr[11:0] + b) % 4096];
        err_pipe[0] <= l1_err_next;
      end
      rv_pipe[1]  <= rv_pipe[0];
      rd_pipe[1]  <= rd_pipe[0];
      err_pipe[1] <= err_pipe[0];
    end
  end

  assign l1_gnt    = l1_gnt_force;
  assign l1_rvalid = rv_pipe[1];
  assign l1_rdata  = rd_pipe[1];

  // one-shot err clear
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n)                        l1_err_next <= 0;
    else if (rv_pipe[0] && l1_err_next) l1_err_next <= 0;

  // =========================================================================
  // DUT
  // =========================================================================
  vlsu_top dut (
    .clk_i              (clk),
    .rst_ni             (rst_n),
    .cmd_valid_i        (cmd_valid),
    .cmd_ready_o        (cmd_ready),
    .cmd_insn_i         (cmd_insn),
    .cmd_base_addr_i    (cmd_base),
    .cmd_stride_bytes_i (cmd_stride),
    .cmd_vl_i           (cmd_vl),
    .cmd_vstart_i       (cmd_vstart),
    .cmd_elem_bytes_i   (cmd_ebytes),
    .cmd_vm_i           (cmd_vm),
    .cmd_mask_i         (cmd_mask),
    .done_valid_o       (done_valid),
    .done_ready_i       (done_ready),
    .exc_valid_o        (exc_valid),
    .exc_badaddr_o      (exc_badaddr),
    .exc_vstart_o       (exc_vstart_out),
    .exc_misaligned_o   (exc_ma),
    .exc_bus_o          (exc_bus),
    .st_valid_i         (st_valid),
    .st_ready_o         (st_ready),
    .st_data_i          (st_data),
    .ld_valid_o         (ld_valid),
    .ld_ready_i         (ld_ready),
    .ld_data_o          (ld_data),
    .idx_valid_i        (idx_valid),
    .idx_ready_o        (idx_ready),
    .idx_data_i         (idx_data),
    .l1_gnt_i           (l1_gnt),
    .l1_rvalid_i        (l1_rvalid),
    .l1_rdata_i         (l1_rdata),
    .l1_req_o           (l1_req),
    .l1_we_o            (l1_we),
    .l1_be_o            (l1_be),
    .l1_addr_o          (l1_addr),
    .l1_wdata_o         (l1_wdata)
  );

  // =========================================================================
  // Waveform dump
  // =========================================================================
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, tb);
  end

  // =========================================================================
  // Load capture buffers  [lane][arrival_index]
  // =========================================================================
  logic [ELEM_W-1:0] ld_cap [0:NrLanes-1][0:63];
  int                ld_cnt [0:NrLanes-1];

  always_ff @(posedge clk)
    for (int ln = 0; ln < NrLanes; ln++)
      if (ld_valid[ln] && ld_ready[ln]) begin
        ld_cap[ln][ld_cnt[ln]] <= ld_data[ln];
        ld_cnt[ln]             <= ld_cnt[ln] + 1;
      end

  // Store address / data capture
  logic [ADDR_W-1:0] st_addr_log [0:63];
  logic [ELEM_W-1:0] st_data_log [0:63];
  int                st_log_cnt;

  always_ff @(posedge clk)
    if (l1_req && l1_we && l1_gnt) begin
      st_addr_log[st_log_cnt] <= l1_addr[ADDR_W-1:0];
      st_data_log[st_log_cnt] <= l1_wdata;
      st_log_cnt              <= st_log_cnt + 1;
    end

  // BE capture (for TC8)
  logic [7:0] be_log [0:63];
  int         be_log_cnt;

  always_ff @(posedge clk)
    if (l1_req && l1_gnt && !l1_we) begin
      be_log[be_log_cnt] <= l1_be;
      be_log_cnt         <= be_log_cnt + 1;
    end

  // =========================================================================
  // Utility: build expected 64-bit word from memory model
  // =========================================================================
  function automatic logic [63:0] mem_word(input int addr);
    logic [63:0] w;
    for (int b = 0; b < 8; b++)
      w[b*8 +: 8] = mem[(addr + b) % 4096];
    return w;
  endfunction

  // =========================================================================
  // Task: clear_all  — reset every driven signal and counters
  // =========================================================================
  task automatic clear_all();
    cmd_valid    = 0;
    cmd_insn     = VLE;
    cmd_base     = '0;
    cmd_stride   = '0;
    cmd_vl       = 0;
    cmd_vstart   = 0;
    cmd_ebytes   = 4'd8;
    cmd_vm       = 1;
    cmd_mask     = '1;
    done_ready   = 1;
    st_valid     = '0;
    st_data      = '0;
    ld_ready     = '1;
    idx_valid    = '0;
    idx_data     = '0;
    l1_gnt_force = 1;
    l1_err_next  = 0;
    st_log_cnt   = 0;
    be_log_cnt   = 0;
    for (int i = 0; i < NrLanes; i++) begin
      ld_cnt[i] = 0;
      for (int j = 0; j < 64; j++) ld_cap[i][j] = '0;
    end
    for (int i = 0; i < 64; i++) begin
      st_addr_log[i] = '0;
      st_data_log[i] = '0;
      be_log[i]      = '0;
    end
  endtask

  // =========================================================================
  // Task: issue_cmd  — handshake command and wait for done/exc
  // =========================================================================
  task automatic issue_cmd(
    input vlsu_insn_e        insn,
    input logic [ADDR_W-1:0] base,
    input logic [ADDR_W-1:0] stride,
    input int                vl,
    input int                vstart,
    input int                ebytes,
    input logic              vm,
    input logic [NrLanes-1:0][7:0] mask,
    input int                timeout = 2000
  );
    int wd;
    @(posedge clk); #1;
    cmd_insn   = insn;   cmd_base   = base;
    cmd_stride = stride; cmd_vl     = vl;
    cmd_vstart = vstart; cmd_ebytes = ebytes;
    cmd_vm     = vm;     cmd_mask   = mask;
    cmd_valid  = 1;

    wd = 0;
    while (!(cmd_valid && cmd_ready)) begin
      @(posedge clk); #1;
      if (++wd > timeout) begin $display("TIMEOUT cmd"); $finish; end
    end
    @(posedge clk); #1;
    cmd_valid = 0;

    wd = 0;
    while (!done_valid) begin
      @(posedge clk); #1;
      if (++wd > timeout) begin $display("TIMEOUT done"); $finish; end
    end
    @(posedge clk); #1;   // consume done pulse (done_ready=1 throughout)
  endtask

  // =========================================================================
  // MAIN
  // =========================================================================
  initial begin
    rst_n = 0;
    clear_all();
    repeat(6) @(posedge clk);
    rst_n = 1;
    repeat(2) @(posedge clk);

    $display("============================================================");
    $display("  VLSU SELF-CHECKING TESTBENCH");
    $display("============================================================");

    // -----------------------------------------------------------------------
    // TC1: VLE unit-stride load — 4 x 64-bit @ 0x100
    // Elements 0-3 → lanes 0-3
    // Addresses: 0x100, 0x108, 0x110, 0x118
    // -----------------------------------------------------------------------
    $display("\n[TC1] VLE unit-stride load  4 x 64-bit  base=0x100");
    begin
      clear_all();
      issue_cmd(VLE, 39'h100, 39'h0, 4, 0, 8, 1, '1);

      `CHECK("TC1.01 no exc",         exc_valid,       1'b0)
      `CHECK("TC1.02 lane0 count=1",  ld_cnt[0],       1)
      `CHECK("TC1.03 lane1 count=1",  ld_cnt[1],       1)
      `CHECK("TC1.04 lane2 count=1",  ld_cnt[2],       1)
      `CHECK("TC1.05 lane3 count=1",  ld_cnt[3],       1)
      `CHECK("TC1.06 lane4 empty",    ld_cnt[4],       0)
      `CHECK("TC1.07 lane0 data",     ld_cap[0][0],    mem_word(32'h100))
      `CHECK("TC1.08 lane1 data",     ld_cap[1][0],    mem_word(32'h108))
      `CHECK("TC1.09 lane2 data",     ld_cap[2][0],    mem_word(32'h110))
      `CHECK("TC1.10 lane3 data",     ld_cap[3][0],    mem_word(32'h118))
    end

    // -----------------------------------------------------------------------
    // TC2: VSE unit-stride store — 4 x 64-bit @ 0x200
    // Lane i writes 0xCAFE_0000_0000_00ii
    // -----------------------------------------------------------------------
    $display("\n[TC2] VSE unit-stride store  4 x 64-bit  base=0x200");
    begin
      clear_all();
      for (int i = 0; i < 4; i++) begin
        st_data[i]  = 64'hCAFE_0000_0000_0000 | i;
        st_valid[i] = 1;
      end

      issue_cmd(VSE, 39'h200, 39'h0, 4, 0, 8, 1, '1);
      st_valid = '0;

      `CHECK("TC2.01 no exc",      exc_valid,       1'b0)
      `CHECK("TC2.02 store cnt",   st_log_cnt,      4)
      `CHECK("TC2.03 addr[0]",     st_addr_log[0],  39'h200)
      `CHECK("TC2.04 addr[1]",     st_addr_log[1],  39'h208)
      `CHECK("TC2.05 addr[2]",     st_addr_log[2],  39'h210)
      `CHECK("TC2.06 addr[3]",     st_addr_log[3],  39'h218)
      `CHECK("TC2.07 wdata[0]",    st_data_log[0],  64'hCAFE_0000_0000_0000)
      `CHECK("TC2.08 wdata[1]",    st_data_log[1],  64'hCAFE_0000_0000_0001)
      `CHECK("TC2.09 wdata[2]",    st_data_log[2],  64'hCAFE_0000_0000_0002)
      `CHECK("TC2.10 wdata[3]",    st_data_log[3],  64'hCAFE_0000_0000_0003)
    end

    // -----------------------------------------------------------------------
    // TC3: VLSE strided load — 4 x 64-bit @ 0x300, stride=16
    // Addresses: 0x300, 0x310, 0x320, 0x330
    // -----------------------------------------------------------------------
    $display("\n[TC3] VLSE strided load  4 x 64-bit  base=0x300  stride=16");
    begin
      clear_all();
      issue_cmd(VLSE, 39'h300, 39'h10, 4, 0, 8, 1, '1);

      `CHECK("TC3.01 no exc",    exc_valid,      1'b0)
      `CHECK("TC3.02 lane0 cnt", ld_cnt[0],      1)
      `CHECK("TC3.03 lane1 cnt", ld_cnt[1],      1)
      `CHECK("TC3.04 lane2 cnt", ld_cnt[2],      1)
      `CHECK("TC3.05 lane3 cnt", ld_cnt[3],      1)
      `CHECK("TC3.06 lane0 dat", ld_cap[0][0],   mem_word(32'h300))
      `CHECK("TC3.07 lane1 dat", ld_cap[1][0],   mem_word(32'h310))
      `CHECK("TC3.08 lane2 dat", ld_cap[2][0],   mem_word(32'h320))
      `CHECK("TC3.09 lane3 dat", ld_cap[3][0],   mem_word(32'h330))
    end

    // -----------------------------------------------------------------------
    // TC4: VSSE strided store — 4 x 64-bit @ 0x400, stride=16
    // -----------------------------------------------------------------------
    $display("\n[TC4] VSSE strided store  4 x 64-bit  base=0x400  stride=16");
    begin
      clear_all();
      for (int i = 0; i < 4; i++) begin
        st_data[i]  = 64'hBEEF_0000_0000_0000 | (i*16);
        st_valid[i] = 1;
      end

      issue_cmd(VSSE, 39'h400, 39'h10, 4, 0, 8, 1, '1);
      st_valid = '0;

      `CHECK("TC4.01 no exc",    exc_valid,       1'b0)
      `CHECK("TC4.02 store cnt", st_log_cnt,      4)
      `CHECK("TC4.03 addr[0]",   st_addr_log[0],  39'h400)
      `CHECK("TC4.04 addr[1]",   st_addr_log[1],  39'h410)
      `CHECK("TC4.05 addr[2]",   st_addr_log[2],  39'h420)
      `CHECK("TC4.06 addr[3]",   st_addr_log[3],  39'h430)
    end

    // -----------------------------------------------------------------------
    // TC5: VLXE indexed load — 4 elements, base=0x500
    // Offsets: lane0→0x00, lane1→0x20, lane2→0x08, lane3→0x40
    // -----------------------------------------------------------------------
    $display("\n[TC5] VLXE indexed load  4 elements  base=0x500");
    begin
      clear_all();
      idx_data[0]  = 64'h00; idx_valid[0] = 1;
      idx_data[1]  = 64'h20; idx_valid[1] = 1;
      idx_data[2]  = 64'h08; idx_valid[2] = 1;
      idx_data[3]  = 64'h40; idx_valid[3] = 1;

      issue_cmd(VLXE, 39'h500, 39'h0, 4, 0, 8, 1, '1);
      idx_valid = '0;

      `CHECK("TC5.01 no exc",    exc_valid,      1'b0)
      `CHECK("TC5.02 lane0 dat", ld_cap[0][0],   mem_word(32'h500 + 32'h00))
      `CHECK("TC5.03 lane1 dat", ld_cap[1][0],   mem_word(32'h500 + 32'h20))
      `CHECK("TC5.04 lane2 dat", ld_cap[2][0],   mem_word(32'h500 + 32'h08))
      `CHECK("TC5.05 lane3 dat", ld_cap[3][0],   mem_word(32'h500 + 32'h40))
    end

    // -----------------------------------------------------------------------
    // TC6: VSXE indexed store — 4 elements, base=0x600
    // Offsets: 0, 16, 32, 48
    // -----------------------------------------------------------------------
    $display("\n[TC6] VSXE indexed store  4 elements  base=0x600");
    begin
      clear_all();
      for (int i = 0; i < 4; i++) begin
        idx_data[i]  = 64'(i * 16);
        idx_valid[i] = 1;
        st_data[i]   = 64'hDEAD_0000_0000_0000 | i;
        st_valid[i]  = 1;
      end

      issue_cmd(VSXE, 39'h600, 39'h0, 4, 0, 8, 1, '1);
      idx_valid = '0; st_valid = '0;

      `CHECK("TC6.01 no exc",    exc_valid,       1'b0)
      `CHECK("TC6.02 store cnt", st_log_cnt,      4)
      `CHECK("TC6.03 addr[0]",   st_addr_log[0],  39'h600)
      `CHECK("TC6.04 addr[1]",   st_addr_log[1],  39'h610)
      `CHECK("TC6.05 addr[2]",   st_addr_log[2],  39'h620)
      `CHECK("TC6.06 addr[3]",   st_addr_log[3],  39'h630)
      `CHECK("TC6.07 wdata[0]",  st_data_log[0],  64'hDEAD_0000_0000_0000)
      `CHECK("TC6.08 wdata[1]",  st_data_log[1],  64'hDEAD_0000_0000_0001)
    end

    // -----------------------------------------------------------------------
    // TC7: VLE 8-bit elements — 4 x 1-byte @ 0x700
    // BE should be 8'h01 for each transaction
    // -----------------------------------------------------------------------
    $display("\n[TC7] VLE 8-bit elements  4 x 1-byte  base=0x700");
    begin
      clear_all();
      issue_cmd(VLE, 39'h700, 39'h0, 4, 0, 1, 1, '1);

      `CHECK("TC7.01 no exc",       exc_valid,   1'b0)
      `CHECK("TC7.02 lane0 cnt",    ld_cnt[0],   1)
      `CHECK("TC7.03 lane1 cnt",    ld_cnt[1],   1)
      `CHECK("TC7.04 lane2 cnt",    ld_cnt[2],   1)
      `CHECK("TC7.05 lane3 cnt",    ld_cnt[3],   1)
      `CHECK("TC7.06 BE[0]=0x01",   be_log[0],   8'h01)
      `CHECK("TC7.07 BE[1]=0x01",   be_log[1],   8'h01)
      `CHECK("TC7.08 BE[2]=0x01",   be_log[2],   8'h01)
      `CHECK("TC7.09 BE[3]=0x01",   be_log[3],   8'h01)
    end

    // -----------------------------------------------------------------------
    // TC8: VLE with mask (vm=0)
    // lane0=0xFF active, lane1=0x00 masked, lane2=0xFF, lane3=0x00 masked
    // Masked lanes → BE=0x00 on the OBI bus
    // -----------------------------------------------------------------------
    $display("\n[TC8] VLE with byte mask  lane1 and lane3 masked (BE=00)");
    begin
      logic [NrLanes-1:0][7:0] tmask;
      clear_all();
      tmask        = '0;
      tmask[0]     = 8'hFF;
      tmask[1]     = 8'h00;
      tmask[2]     = 8'hFF;
      tmask[3]     = 8'h00;

      issue_cmd(VLE, 39'h800, 39'h0, 4, 0, 8, 0, tmask);

      `CHECK("TC8.01 no exc",        exc_valid,   1'b0)
      `CHECK("TC8.02 lane0 BE=FF",   be_log[0],   8'hFF)
      `CHECK("TC8.03 lane1 BE=00",   be_log[1],   8'h00)
      `CHECK("TC8.04 lane2 BE=FF",   be_log[2],   8'hFF)
      `CHECK("TC8.05 lane3 BE=00",   be_log[3],   8'h00)
    end

    // -----------------------------------------------------------------------
    // TC9: VLE vstart=2 (restart) — 2 elements starting at elem index 2
    // Only lanes 2 and 3 should receive data
    // Addresses: base + 2*8 = 0x910, base + 3*8 = 0x918
    // -----------------------------------------------------------------------
    $display("\n[TC9] VLE vstart=2 restart  vl=2  base=0x900");
    begin
      clear_all();
      issue_cmd(VLE, 39'h900, 39'h0, 2, 2, 8, 1, '1);

      `CHECK("TC9.01 no exc",      exc_valid,       1'b0)
      `CHECK("TC9.02 lane0 empty", ld_cnt[0],       0)
      `CHECK("TC9.03 lane1 empty", ld_cnt[1],       0)
      `CHECK("TC9.04 lane2 cnt=1", ld_cnt[2],       1)
      `CHECK("TC9.05 lane3 cnt=1", ld_cnt[3],       1)
      `CHECK("TC9.06 lane4 empty", ld_cnt[4],       0)
      `CHECK("TC9.07 lane2 dat",   ld_cap[2][0],    mem_word(32'h910))
      `CHECK("TC9.08 lane3 dat",   ld_cap[3][0],    mem_word(32'h918))
    end

    // -----------------------------------------------------------------------
    // TC10: Misalignment exception
    // base=0xA01 (unaligned), elem_bytes=2 — must fault, no OBI request issued
    // -----------------------------------------------------------------------
    $display("\n[TC10] Misalignment exception  base=0xA01  ebytes=2");
    begin
      clear_all();
      issue_cmd(VLE, 39'hA01, 39'h0, 4, 0, 2, 1, '1);

      `CHECK("TC10.01 exc_valid=1",     exc_valid,   1'b1)
      `CHECK("TC10.02 exc_ma=1",        exc_ma,      1'b1)
      `CHECK("TC10.03 exc_bus=0",       exc_bus,     1'b0)
      `CHECK("TC10.04 badaddr=0xA01",   exc_badaddr, 39'hA01)
      `CHECK("TC10.05 no OBI request",  be_log_cnt,  0)
      `CHECK("TC10.06 no ld delivered", ld_cnt[0],   0)
    end

    // -----------------------------------------------------------------------
    // TC11: Bus error exception
    // Inject error on first read response
    // -----------------------------------------------------------------------
    $display("\n[TC11] Bus error exception  base=0xB00");
    begin
      clear_all();
      l1_err_next = 1;

      issue_cmd(VLE, 39'hB00, 39'h0, 4, 0, 8, 1, '1);

      `CHECK("TC11.01 exc_valid=1",  exc_valid,   1'b1)
      `CHECK("TC11.02 exc_bus=1",    exc_bus,     1'b1)
      `CHECK("TC11.03 exc_ma=0",     exc_ma,      1'b0)
      `CHECK("TC11.04 badaddr",      exc_badaddr, 39'hB00)
    end

    // -----------------------------------------------------------------------
    // TC12: VSE back-pressure — lane holds st_valid low for 3 cycles
    // LSU must stay in S_COLLECT_ST until data arrives
    // -----------------------------------------------------------------------
    $display("\n[TC12] VSE back-pressure  lane stalls 3 cycles  base=0xC00");
    begin
      clear_all();
      st_valid = '0;

      fork
        // Lane driver: each element delayed by 3 cycles
        begin : lane_driver
          for (int e = 0; e < 4; e++) begin
            int ln;
            ln = e % NrLanes;
            repeat(3) @(posedge clk); #1;
            st_data[ln]  = 64'hF00D_0000_0000_0000 | e;
            st_valid[ln] = 1;
            @(posedge clk iff st_ready[ln]); #1;
            st_valid[ln] = 0;
          end
        end
        begin : cmd_driver
          issue_cmd(VSE, 39'hC00, 39'h0, 4, 0, 8, 1, '1);
        end
      join

      `CHECK("TC12.01 no exc",    exc_valid,        1'b0)
      `CHECK("TC12.02 store cnt", st_log_cnt,       4)
      `CHECK("TC12.03 addr[0]",   st_addr_log[0],   39'hC00)
      `CHECK("TC12.04 addr[1]",   st_addr_log[1],   39'hC08)
      `CHECK("TC12.05 addr[2]",   st_addr_log[2],   39'hC10)
      `CHECK("TC12.06 addr[3]",   st_addr_log[3],   39'hC18)
      `CHECK("TC12.07 wdata[0]",  st_data_log[0],   64'hF00D_0000_0000_0000)
      `CHECK("TC12.08 wdata[1]",  st_data_log[1],   64'hF00D_0000_0000_0001)
    end

    // =======================================================================
    // Final report
    // =======================================================================
    repeat(5) @(posedge clk);
    $display("\n============================================================");
    $display("  RESULTS : %0d / %0d passed", tests_pass, tests_run);
    if (tests_fail == 0) $display("  *** ALL TESTS PASSED ***");
    else                 $display("  *** %0d TEST(S) FAILED ***", tests_fail);
    $display("============================================================\n");
    $finish;
  end

  // Watchdog
  initial begin
    #1_000_000;
    $display("[WATCHDOG] Simulation timeout — force exit");
    $finish;
  end

endmodule
