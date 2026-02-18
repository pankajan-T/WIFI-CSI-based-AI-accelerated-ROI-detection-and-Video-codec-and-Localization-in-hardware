`timescale 1ps/1ps

module tb_mpeg2encoder ();

localparam XL = 7;
localparam YL = 6;

`define VIDEO2_IN_YUV_RAW_FILE  "../../../../../SIM/data/640x320.yuv"
`define VIDEO2_OUT_MPEG2_FILE   "../../../../../SIM/data/640x320.m2v"
`define VIDEO2_XSIZE  640
`define VIDEO2_YSIZE  320

`define VIDEO3_IN_YUV_RAW_FILE  "../../../../../SIM/data/1440x704.yuv"
`define VIDEO3_OUT_MPEG2_FILE   "../../../../../SIM/data/1440x704.m2v"
`define VIDEO3_XSIZE  1440
`define VIDEO3_YSIZE  704

// NOTE: Huge arrays; OK for sim but heavy on memory
reg [7:0] frameY [0:2047] [0:2047];
reg [7:0] frameU [0:2047] [0:2047];
reg [7:0] frameV [0:2047] [0:2047];

reg rstn = 1'b1;
reg clk  = 1'b0;
always #5000 clk = ~clk;

reg          i_sequence_stop = 0;
wire         o_sequence_busy;

reg  [ XL:0] i_xsize16;
reg  [ YL:0] i_ysize16;

// ---------------- Pixel stream (NOW READY/VALID) ----------------
reg          i_pix_tvalid = 0;
wire         o_pix_tready;

reg  [  7:0] i_Y0, i_Y1, i_Y2, i_Y3;
reg  [  7:0] i_U0, i_U1, i_U2, i_U3;
reg  [  7:0] i_V0, i_V1, i_V2, i_V3;

// ---------------- Output MPEG2 stream ----------------
wire         o_en;
wire         o_last;
wire [255:0] o_data;

// ---------------- ROI handshake (encoder requests once per frame) ----------------
wire         o_roi_tready;   // encoder requests ROI when high
reg          i_roi_tvalid;   // TB provides ROI when high
reg  [XL-1:0] i_roi_x16_min;
reg  [XL-1:0] i_roi_x16_max;
reg  [YL-1:0] i_roi_y16_min;
reg  [YL-1:0] i_roi_y16_max;

wire roi_fire = o_roi_tready & i_roi_tvalid;

// DUT
mpeg2encoder #(
    .XL           ( XL ),
    .YL           ( YL ),
    .VECTOR_LEVEL ( 1  ),
    .Q_LEVEL      ( 1 ),
    .Q_LEVEL_BG   (5)
) mpeg2encoder_i (
    .rstn            ( rstn ),
    .clk             ( clk  ),

    .i_xsize16       ( i_xsize16 ),
    .i_ysize16       ( i_ysize16 ),
    .i_pframes_count ( 8'd23 ),

    .i_pix_tvalid    ( i_pix_tvalid ),
    .o_pix_tready    ( o_pix_tready ),
    .i_Y0            ( i_Y0 ), .i_Y1 ( i_Y1 ), .i_Y2 ( i_Y2 ), .i_Y3 ( i_Y3 ),
    .i_U0            ( i_U0 ), .i_U1 ( i_U1 ), .i_U2 ( i_U2 ), .i_U3 ( i_U3 ),
    .i_V0            ( i_V0 ), .i_V1 ( i_V1 ), .i_V2 ( i_V2 ), .i_V3 ( i_V3 ),

    .i_sequence_stop ( i_sequence_stop ),
    .o_sequence_busy ( o_sequence_busy ),

    .o_en            ( o_en ),
    .o_last          ( o_last ),
    .o_data          ( o_data ),

    .o_roi_tready    ( o_roi_tready ),
    .i_roi_tvalid    ( i_roi_tvalid ),
    .i_roi_x16_min   ( i_roi_x16_min ),
    .i_roi_x16_max   ( i_roi_x16_max ),
    .i_roi_y16_min   ( i_roi_y16_min ),
    .i_roi_y16_max   ( i_roi_y16_max )
);

integer fp_in, fp_out;
integer xsize, ysize;
integer num_video;
integer f, y, x, i;
integer frame_index;

// clamp helper
function automatic integer clamp_int(input integer v, input integer lo, input integer hi);
begin
    if (v < lo) clamp_int = lo;
    else if (v > hi) clamp_int = hi;
    else clamp_int = v;
end
endfunction

task automatic update_roi_for_frame(
    input integer fidx,
    input integer mb_w,   // number of MBs in width  (xsize/16)
    input integer mb_h    // number of MBs in height (ysize/16)
);
    integer group;
    integer x0,x1,y0,y1;
begin
    group = (fidx / 5);   // changes every 5 frames

    // Define some moving ROIs in macroblock coordinates (x16,y16)
    case (group % 4)
        0: begin // ROI 0: top-left small region
            x0 = 0;  x1 = 3;
            y0 = 0;  y1 = 2;
        end
        1: begin // ROI 1: center region
            x0 = (mb_w/2) - 5;  x1 = (mb_w/2) + 5;
            y0 = (mb_h/2) - 3;  y1 = (mb_h/2) + 3;
        end
        2: begin // ROI 2: right vertical strip
            x0 = mb_w - 8;  x1 = mb_w - 1;
            y0 = 0;         y1 = mb_h - 1;
        end
        default: begin // ROI 3: bottom band
            x0 = 0;         x1 = mb_w - 1;
            y0 = mb_h - 5;  y1 = mb_h - 1;
        end
    endcase

    // Clamp to valid macroblock indices
    x0 = clamp_int(x0, 0, mb_w-1);
    x1 = clamp_int(x1, 0, mb_w-1);
    y0 = clamp_int(y0, 0, mb_h-1);
    y1 = clamp_int(y1, 0, mb_h-1);

    if (x1 < x0) x1 = x0;
    if (y1 < y0) y1 = y0;

    i_roi_x16_min = x0[XL-1:0];
    i_roi_x16_max = x1[XL-1:0];
    i_roi_y16_min = y0[YL-1:0];
    i_roi_y16_max = y1[YL-1:0];
end
endtask

// Push one 4-pixel group with backpressure
task automatic send_pix_group(
    input [7:0] y0, y1, y2, y3,
    input [7:0] u0, u1, u2, u3,
    input [7:0] v0, v1, v2, v3
);
begin
    // Drive payload + valid, hold until accepted
    i_pix_tvalid <= 1'b1;
    {i_Y0, i_Y1, i_Y2, i_Y3} <= {y0,y1,y2,y3};
    {i_U0, i_U1, i_U2, i_U3} <= {u0,u1,u2,u3};
    {i_V0, i_V1, i_V2, i_V3} <= {v0,v1,v2,v3};

    // Wait for acceptance
    while (!(i_pix_tvalid && o_pix_tready))
        @(posedge clk);

    // Deassert valid next cycle (or you can keep streaming, but simplest is 1-beat pulses)
    @(posedge clk);
    i_pix_tvalid <= 1'b0;
end
endtask

initial begin
    // init
    i_pix_tvalid <= 1'b0;
    i_sequence_stop <= 1'b0;

    i_roi_tvalid <= 1'b0;
    i_roi_x16_min <= 0;
    i_roi_x16_max <= 0;
    i_roi_y16_min <= 0;
    i_roi_y16_max <= 0;

    frame_index  <= 0;

    repeat(4) @(posedge clk);
    rstn <= 1'b0;
    repeat(4) @(posedge clk);
    rstn <= 1'b1;
    @(posedge clk);

    for (num_video=2; num_video<=3; num_video=num_video+1) begin

        case (num_video)
            2: begin
                fp_in  = $fopen(`VIDEO2_IN_YUV_RAW_FILE, "rb");
                fp_out = $fopen(`VIDEO2_OUT_MPEG2_FILE , "wb");
                xsize  = `VIDEO2_XSIZE;
                ysize  = `VIDEO2_YSIZE;
            end
            3: begin
                fp_in  = $fopen(`VIDEO3_IN_YUV_RAW_FILE, "rb");
                fp_out = $fopen(`VIDEO3_OUT_MPEG2_FILE , "wb");
                xsize  = `VIDEO3_XSIZE;
                ysize  = `VIDEO3_YSIZE;
            end
        endcase

        $display("start to encode video %1d (%4dx%4d)", num_video, xsize, ysize);

        if (fp_in == 0 || fp_out == 0) begin
            $display("*** couldn't open input/output file");
            $finish;
        end

        i_xsize16 <= xsize / 16;
        i_ysize16 <= ysize / 16;

        fork
            // thread : push raw pixels (ROI then pixels, per frame)
            begin : PUSH_THREAD
                integer mb_w, mb_h;
                mb_w = xsize / 16;
                mb_h = ysize / 16;

                // load first frame
                for (y=0; y<ysize; y=y+1)
                    for (x=0; x<xsize; x=x+1)
                        frameY[y][x] = $fgetc(fp_in);
                for (y=0; y<ysize; y=y+1)
                    for (x=0; x<xsize; x=x+1)
                        frameU[y][x] = $fgetc(fp_in);
                for (y=0; y<ysize; y=y+1)
                    for (x=0; x<xsize; x=x+1)
                        frameV[y][x] = $fgetc(fp_in);

                for (f=0; !$feof(fp_in); f=f+1) begin
                    $display("  start to encode video %1d frame %3d", num_video, f);

                    // Update ROI payload for this frame
                    update_roi_for_frame(frame_index, mb_w, mb_h);

                    // Wait until encoder requests ROI (o_roi_tready)
                    while (!o_roi_tready)
                        @(posedge clk);

                    // Present ROI payload and hold valid until handshake
                    i_roi_tvalid <= 1'b1;
                    while (!roi_fire)
                        @(posedge clk);
                    @(posedge clk);
                    i_roi_tvalid <= 1'b0;

                    // Now stream pixels with backpressure
                    for (y=0; y<ysize; y=y+1) begin
                        for (x=0; x<xsize; x=x+4) begin
                            send_pix_group(
                                frameY[y][x],   frameY[y][x+1], frameY[y][x+2], frameY[y][x+3],
                                frameU[y][x],   frameU[y][x+1], frameU[y][x+2], frameU[y][x+3],
                                frameV[y][x],   frameV[y][x+1], frameV[y][x+2], frameV[y][x+3]
                            );
                        end
                    end

                    frame_index = frame_index + 1;

                    // load next frame
                    for (y=0; y<ysize; y=y+1)
                        for (x=0; x<xsize; x=x+1)
                            frameY[y][x] = $fgetc(fp_in);
                    for (y=0; y<ysize; y=y+1)
                        for (x=0; x<xsize; x=x+1)
                            frameU[y][x] = $fgetc(fp_in);
                    for (y=0; y<ysize; y=y+1)
                        for (x=0; x<xsize; x=x+1)
                            frameV[y][x] = $fgetc(fp_in);
                end

                // request sequence stop
                i_sequence_stop <= 1'b1;
                @(posedge clk);
                i_sequence_stop <= 1'b0;
                @(posedge clk);
            end

            // thread : write stream to file
            begin : WRITE_THREAD
                while (~o_sequence_busy)
                    @(posedge clk);
                while (o_sequence_busy) begin
                    if (o_en)
                        for(i=0; i<32; i=i+1)
                            $fwrite(fp_out, "%c", o_data[i*8 +: 8]);
                    @(posedge clk);
                end
            end
        join

        $fclose(fp_in);
        $fclose(fp_out);
        $display("end of video %1d", num_video);
    end

    $finish;
end

endmodule
