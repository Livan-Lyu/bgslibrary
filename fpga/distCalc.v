module distCalc #(
    parameter integer HISTORY_COUNT = 20,
    parameter integer PIXEL_BITS = 32,
    parameter integer OUTPUT_WORD_BITS = 32
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         in_valid,
    input  wire [PIXEL_BITS-1:0]        current_pixel,
    input  wire [HISTORY_COUNT*PIXEL_BITS-1:0] history_pixels,
    input  wire [31:0]                  threshold,
    output reg                          out_valid,
    output reg  [OUTPUT_WORD_BITS-1:0]  comparison_word
);

    // BGRX32 layout:
    // current_pixel[7:0]   = B
    // current_pixel[15:8]  = G
    // current_pixel[23:16] = R
    // current_pixel[31:24] = X
    //
    // comparison_word[k] corresponds to history_pixels[k*32 +: 32].
    // Low HISTORY_COUNT bits are valid; upper bits are reserved and forced to 0.
    // Threshold semantics:
    // |B - Bk| + |G - Gk| + |R - Rk| <= threshold

    function [7:0] abs_diff_8;
        input [7:0] lhs;
        input [7:0] rhs;
        begin
            if (lhs >= rhs) begin
                abs_diff_8 = lhs - rhs;
            end else begin
                abs_diff_8 = rhs - lhs;
            end
        end
    endfunction

    wire [HISTORY_COUNT-1:0] match_bits;

    genvar idx;
    generate
        for (idx = 0; idx < HISTORY_COUNT; idx = idx + 1) begin : gen_compare
            wire [PIXEL_BITS-1:0] history_pixel;
            wire [7:0] db;
            wire [7:0] dg;
            wire [7:0] dr;
            wire [10:0] sad_rgb;

            assign history_pixel = history_pixels[(idx * PIXEL_BITS) +: PIXEL_BITS];
            assign db = abs_diff_8(current_pixel[7:0], history_pixel[7:0]);
            assign dg = abs_diff_8(current_pixel[15:8], history_pixel[15:8]);
            assign dr = abs_diff_8(current_pixel[23:16], history_pixel[23:16]);
            assign sad_rgb = {3'b000, db} + {3'b000, dg} + {3'b000, dr};
            assign match_bits[idx] = (sad_rgb <= threshold[10:0]);
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            comparison_word <= {OUTPUT_WORD_BITS{1'b0}};
        end else begin
            out_valid <= in_valid;

            if (in_valid) begin
                comparison_word <= {{(OUTPUT_WORD_BITS - HISTORY_COUNT){1'b0}}, match_bits};
            end
        end
    end

endmodule
