// cpu64_obi_receiver.v - Small OBI receiver shim for L1 cache integration
`timescale 1ns/1ps

module cpu64_obi_receiver #(
    parameter CORE_ADDR_W = 39,
    parameter DATA_W      = 64
) (
    // Clocks/Reset
    input                  clk_i,
    input                  rst_ni,

    // Core-side OBI (host driver facing)
    input                  req_i,
    input                  we_i,
    input        [7:0]     be_i,
    input  [CORE_ADDR_W-1:0] addr_i,
    input  [DATA_W-1:0]    wdata_i,
    output reg             gnt_o,
    output reg             rvalid_o,
    output reg [DATA_W-1:0] rdata_o,

    // L1 CPU-side interface
    output reg             l1_req_o,
    output reg             l1_we_o,
    output reg     [7:0]   l1_be_o,
    output reg    [63:0]   l1_addr_o,
    output reg    [63:0]   l1_wdata_o,
    input                  l1_gnt_i,
    input                  l1_rvalid_i,
    input        [63:0]    l1_rdata_i
);

    // Simple one-outstanding request tracker
    reg outstanding_q, outstanding_n;
    reg we_q, we_n;

    // Default combinational
    always @(*) begin
        // Drive-through to L1 each cycle
        l1_req_o   = req_i && (~outstanding_q);
        l1_we_o    = we_i;
        l1_be_o    = be_i;
        l1_addr_o  = { { (64-CORE_ADDR_W){1'b0} }, addr_i };
        l1_wdata_o = wdata_i;

        // Grant semantics: only grant to core when L1 grants (proper OBI protocol)
        gnt_o      = l1_gnt_i;

        // Response from L1 is forwarded as-is
        rvalid_o   = l1_rvalid_i;
        rdata_o    = l1_rdata_i;

        // Track outstanding only for reads (stores complete on accept)
        we_n           = we_i;
        outstanding_n  = outstanding_q;

        // On a new accepted read request (when L1 grants), mark outstanding
        if (l1_gnt_i && l1_req_o && (~we_i)) begin
            outstanding_n = 1'b1;
        end
        // When read data arrives, clear outstanding
        if (l1_rvalid_i) begin
            outstanding_n = 1'b0;
        end
    end

    // Sequential
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            outstanding_q <= 1'b0;
            we_q          <= 1'b0;
        end else begin
            outstanding_q <= outstanding_n;
            we_q          <= we_n;
        end
    end

`ifdef CPU64_DBG_TRACE
    //===================== Debug Trace ==========================//
    always @(posedge clk_i) begin
        if (rst_ni) begin
            if (req_i && gnt_o) begin
                $display("[OBI_RX] core_req we=%0b addr=0x%0h be=0x%02h wdata=0x%016h l1_gnt=%0b rvalid=%0b outstanding=%0b", 
                         we_i, addr_i, be_i, wdata_i, l1_gnt_i, l1_rvalid_i, outstanding_q);
            end
        end
    end
`endif

endmodule


