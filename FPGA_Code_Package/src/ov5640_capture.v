//==============================================================================
// 文件�? : ov5640_capture.v
// 模块�? : ov5640_capture
// 描述   : OV5640 DVP 接口（RGB565 模式）视频采集模块，输出 AXI4-Stream 视频�?
//------------------------------------------------------------------------------
// 工具兼容 : Verilog-2001（依�? $clog2 系统函数，Vivado / Quartus / Icarus 均支持）
//==============================================================================
//
// 【功能概述�??
//   OV5640 配置�? DVP 并行输出、RGB565 格式时，每个像素�? 2 个字节组成，
//   每个字节占用 1 �? pclk 周期；高字节（第 1 字节）在前，低字节（�? 2 字节）在后：
//
//       高字�?(�?) : {R[4:0], G[5:3]}
//       低字�?(�?) : {G[2:0], B[4:0]}
//
//   本模块在 pclk 上升沿且 href 为高时采�? data_in，用�?�? 2 状�?�字节相�?
//   状�?�机把连续两个字节拼�? 16 �? RGB565，再补零扩展�? 24 �? RGB888�?
//   经内�? FWFT 同步 FIFO 缓冲后，以标�? AXI4-Stream 握手输出给下游�??
//
// 【数据�?�路框图�?
//
//        pclk / href / vsync / data_in[7:0]
//                        �?
//                        �?
//          ┌─�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?     ┌─�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?
//          �?  字节相位状�?�机         �?     �?  输出延迟�?�?          �?
//          �?  S_HIGH �? S_LOW       �? �?�?�? �?  pixel_rgb565        �?
//          �?  �?2 字节 �? 1 像素�?    �?     �?  push_pending        �?
//          └─�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?     └─�?�?�?�?�?�?�?�?�?┬─�?�?�?�?�?�?�?�?�?�?�?
//                                                   �?
//                                                   �?
//          ┌─�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?     ┌─�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?
//          �?  FWFT 同步 FIFO        �? �?�?�? �?  AXI4-Stream 输出      �?
//          �?  26bit × FIFO_DEPTH   �?     �?  tvalid / tready /     �?
//          �?  {SOF, tlast, RGB888} �?     �?  tdata / tlast / tuser �?
//          └─�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?     └─�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?
//
// 【字节拼接时序�??
//
//   pclk      : _/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_
//   href      : ‾�?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?�\__
//   data_in   : [R4..0 G5..3][G2..0 B4..0][R4..0 G5..3][G2..0 B4..0]
//   byte_phase:   S_HIGH      S_LOW       S_HIGH      S_LOW
//   像素拼合  :                P0 完成                   P1 完成
//   push_pending �? P0 完成后的下一拍拉高一拍，该拍�? P0 写入 FIFO
//
// 【背压处理（本模块的核心设计）�??
//   摄像头无法被反向暂停：href 有效期间字节�? pclk 节奏照常到来，本模块
//   必须全部接收。因此当 AXI 下游 tready=0 时：
//     (1) 采集侧照常把拼好的像素写入内�? FIFO�?
//     (2) AXI �? tvalid 保持、tdata/tlast 保持稳定，暂停弹�?
//         —�?? �?"暂停输出、等待下游就�?"，不丢数据；
//     (3) 只有当下游停顿超�? FIFO 容量（按像素数计）时才会溢出�?
//         此时 fifo_overflow 拉高报警（粘滞，�?复位才能清除）�??
//   默认 FIFO_DEPTH=2048 �? 1280 宽一整行：下游在�?整行时间内不握手也不会丢
//   数据。若�?要承受多行甚至整帧的停顿，请加大 FIFO_DEPTH（注意资源开�?�?
//   或把 DMA 挂到大容量外部存储（�? DDR）�??
//
// 【AXI4-Stream 语义说明�?
//   - �?次传输发生在 tvalid �? tready 同时为高的时钟沿�?
//   - tvalid 为高�? tready 为低时，tdata/tlast/tuser 必须保持稳定不变�?
//   - tlast 必须�?"�?后一个传�?"同时有效，不能单独在行消隐期拉高；因此本
//     模块�?"行结�?"信息随行内最后一个像素一起存�? FIFO，在该像素被下游
//     接收的那�?拍输�? tlast=1�?
//   - tuser[0] 为帧起始（SOF）标记，与帧内第�?个像素同时有效，符合 Xilinx
//     AXI4-Stream 视频协议惯例（对�? VDMA 时可直接�? s_axis_video_tuser）�??
//==============================================================================
    `timescale 1ns / 1ps
    `default_nettype none
module ov5640_capture #(
    //--------------------------------------------------------------------
    // FIFO 深度（像素数），必须�? 2 的幂�?
    // 建议 �? �?大行宽（像素数）�?1280 �? �? 2048�?2592 �? �? 4096�?
    // 深度越大，能承受的下游停顿时间越长，�? LUT/BRAM �?�?也越大�??
    //--------------------------------------------------------------------
    parameter integer FIFO_DEPTH = 512,

    //--------------------------------------------------------------------
    // RGB565 �? 24 位输出的扩展方式�?
    //   0（默认）：�?��?�道低位补零，得到标�? RGB888�?
    //             R[7:0]={R[4:0],3'b000}  G[7:0]={G[5:0],2'b00}  B[7:0]={B[4:0],3'b000}
    //   1       ：直通低位对�? {8'd0, RGB565}，供特殊后端使用
    //--------------------------------------------------------------------
    parameter integer RGB_EXTEND = 0
) (
    //==================== DVP 摄像头接�? ====================
    input  wire        pclk,        // 像素时钟，模块内�?有时序�?�辑同步于其上升�?
    input  wire        rst_n,       // 低电平有效异步复位（建议外部做去�?/复位释放同步�?
    input  wire        vsync,       // 帧同步，高电�? = 帧有效期�?
    input  wire        href,        // 行同步，高电�? = 行数据有�?
    input  wire [7:0]  data_in,     // 8 �? DVP 数据总线（RGB565，每像素 2 字节�?

    //==================== AXI4-Stream 主机接口 ====================
    (* X_INTERFACE_IGNORE = "true" *) output wire        m_axis_tvalid,  // 数据有效�?= FIFO 非空�?
    (* X_INTERFACE_IGNORE = "true" *) input  wire        m_axis_tready,  // 下游就绪；tvalid&&tready 同时为高完成�?次传�?
    (* X_INTERFACE_IGNORE = "true" *) output wire [31:0] m_axis_tdata,   // 24 �? RGB888 像素 {R[7:0],G[7:0],B[7:0]}
    (* X_INTERFACE_IGNORE = "true" *) output wire        m_axis_tlast,   // 行（包）结束标志，与行内�?后一个像素同时有�?
    (* X_INTERFACE_IGNORE = "true" *)output wire        m_axis_tuser,   // 帧起始标�? SOF，与帧内第一个像素同时有效（可�?�，可悬空）

    //==================== 状�?? / 调试信号 ====================
    output reg         fifo_overflow,  // 粘滞上溢报警：曾因下游停顿过久丢过像素（复位清除�?
    output wire [$clog2(FIFO_DEPTH):0] fifo_usedw    // FIFO 当前占用深度（调试用，不�?要可悬空�?
);

    //====================== 内部常量 ======================
    localparam FIFO_AW = $clog2(FIFO_DEPTH);   // FIFO 地址位宽
    localparam FIFO_W  = 34;                   // FIFO 字宽 = {tuser(1), tlast(1), RGB888(24)}

    //---- 字节相位状�?�机：状态编码（1 bit 状�?�机�?----
    localparam S_HIGH = 1'b0;   // 等待/接收像素的高字节（第 1 字节�?
    localparam S_LOW  = 1'b1;   // 接收像素的低字节（第 2 字节�?

    //====================== 内部寄存�? ======================
    reg          byte_phase;     // 状�?�机现�?�：S_HIGH / S_LOW
    reg [7:0]    byte_high_r;    // 已锁存的高字节（等待与低字节拼合�?
    reg [15:0]   pixel_rgb565;   // 拼合完成�? 16 位像素（等待写入 FIFO�?
    reg          push_pending;   // =1：pixel_rgb565 有效，下�?拍写�? FIFO
    reg          href_d;         // href 延迟 1 拍（用于边沿�?测）
    reg          vsync_d;        // vsync 延迟 1 拍（用于边沿�?测）
    reg          frame_sof;      // 帧起始标记：=1 时下�?个入队像素携�? tuser(SOF)

    //================ RGB565 �? 24 位扩展（组合逻辑�?================
    // �?"推入 FIFO �?"就把 16 位像素扩�? 24 位，FIFO 里直接存 RGB888�?
    wire [23:0] rgb565_to_rgb888;
    generate
        if (RGB_EXTEND == 0) begin : gen_rgb888
            // 标准做法：每个颜色�?�道低位补零
            //   R = {RGB565[15:11], 3'b000}
            //   G = {RGB565[10:5],  2'b00 }
            //   B = {RGB565[4:0],   3'b000}
            assign rgb565_to_rgb888 = {pixel_rgb565[15:11], 3'b000,   // R
                                       pixel_rgb565[10: 5], 2'b00,    // G
                                       pixel_rgb565[ 4: 0], 3'b000};  // B
        end else begin : gen_rgb565_pad
            // 低位对齐直�?�（仅当后端期望 {8'd0, RGB565} 时�?�用�?
            assign rgb565_to_rgb888 = {8'd0, pixel_rgb565};
        end
    endgenerate

    //====================== FWFT 同步 FIFO ======================
    (* ram_style = "block" *)
    reg [FIFO_W-1:0] fifo_mem [0:FIFO_DEPTH-1];   // 存储体（寄存器阵列）
    reg [FIFO_AW:0]  fifo_wptr;                   // 写指针：�? 1 �?"圈数"位，用于判满
    reg [FIFO_AW:0]  fifo_rptr;                   // 读指针：�? 1 �?"圈数"位，用于判空
    reg              fifo_dv;                     // 输出数据有效（直接驱�? m_axis_tvalid�?
    reg [FIFO_W-1:0] fifo_dout;                   // 输出数据寄存器（�? tlast/tuser 位）

    //---- 判满：低位地�?相同而圈数位不同 �? 写指针已领先整整�?�? ----
    wire fifo_full = (fifo_wptr[FIFO_AW-1:0] == fifo_rptr[FIFO_AW-1:0]) &&
                     (fifo_wptr[FIFO_AW]     != fifo_rptr[FIFO_AW]);

    //---- FIFO 输入（组合�?�辑拼接�?----
    // 入队时刻判定 tlast：如�?"推出这一�?"href 已经拉低（或 vsync 拉低），
    // 说明这个像素是行的最后一个像素（帧的�?后一行的�?后一个像素同样满足）�?
    // 于是�? tlast 与像素一起存�? FIFO，保证输出时 tlast 与数据严格对齐�??
    wire [FIFO_W-1:0] fifo_din = {frame_sof, ((href == 1'b0) || (vsync == 1'b0)), 8'd0, rgb565_to_rgb888};  // [23:0] RGB888

    wire fifo_wr_en = push_pending && !fifo_full;      // 写使能：有像素待推且未满
    wire fifo_rd_en = m_axis_tvalid && m_axis_tready;  // 读使能：�?次成功的 AXI 握手

    //---- 指针次�?�（组合，统�?在时钟沿更新�?----
    wire [FIFO_AW:0] fifo_wptr_next = fifo_wptr + {{(FIFO_AW){1'b0}}, fifo_wr_en};
    wire [FIFO_AW:0] fifo_rptr_next = fifo_rptr + {{(FIFO_AW){1'b0}}, fifo_rd_en};

    //--------------------------------------------------------------------------
    // 采集流程：字节相位状态机 + 入队控制
    //--------------------------------------------------------------------------
    // 状�?�机跳转逻辑（全部在 pclk 上升沿评估）�?
    //
    //   现�??      |  转移条件                 |  动作                            |  次�??
    //   ----------+--------------------------+---------------------------------+--------
    //   S_HIGH    |  href == 1               |  锁存 data_in �? byte_high_r      |  S_LOW
    //   S_HIGH    |  href == 0               |  无（行消隐，等待新行�?           |  S_HIGH
    //   S_LOW     |  href == 1               |  拼合 {byte_high_r, data_in} �?   |  S_HIGH
    //             |                          |  pixel_rgb565；push_pending=1    |
    //   S_LOW     |  href == 0（异常：        |  低字节补 8'h00 强行拼合�?       |  S_HIGH
    //             |  本行出现奇数个字节）      |  push_pending=1（携 tlast�?      |
    //
    // 为什么需�? push_pending 延迟�?拍再写入 FIFO�?
    //   �? 2 字节完成拼合的当拍，我们还不知道这个像素是不�?"行的�?后一�?"—�??
    //   href 的下降沿要等到下�?拍才看得见�?�因此让拼好的像素在
    //   pixel_rgb565 里多停留�?拍，下一拍观�? href：若已拉低，说明它就�?
    //   行尾像素，入队时�? tlast �?并写�? FIFO�?
    //   时序保证：push 期间（push_pending=1 的这�?拍）byte_phase 必然处于
    //   S_HIGH（拼合完成的当拍状�?�机已回�? S_HIGH），因此这一拍即使仍�?
    //   行内、采到了下一个像素的高字节，也只�? byte_high_r，不会破�?
    //   pixel_rgb565，FIFO 输入端数据在整个 push 拍保持稳定�??
    //--------------------------------------------------------------------------
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            byte_phase    <= S_HIGH;
            byte_high_r   <= 8'd0;
            pixel_rgb565  <= 16'd0;
            push_pending  <= 1'b0;
            href_d        <= 1'b0;
            vsync_d       <= 1'b0;
            frame_sof     <= 1'b0;
            fifo_overflow <= 1'b0;
        end else begin
            //---- �? 0 步：延迟�?拍，供边沿检测使�? ----
            href_d  <= href;
            vsync_d <= vsync;

            //---- �? 1 步：把上�?拍拼好的像素推入 FIFO ----
            // �? FIFO 写�?�辑共用 fifo_full / fifo_wr_en �?"拍前�?"�?
            // 两�?�在同一时钟沿采样到完全�?致的条件，保证不会错拍�??
            if (push_pending) begin
                if (!fifo_full) begin
                    push_pending <= 1'b0;             // 像素已被 FIFO 接收
                    if (frame_sof) frame_sof <= 1'b0; // SOF 已随该像素输出，清除标记
                end else begin
                    push_pending  <= 1'b0;            // 缓冲耗尽，只能丢弃该像素
                    fifo_overflow <= 1'b1;            // 拉高粘滞上溢报警
                end
            end

            //---- �? 2 步：vsync 上升�? �? 新帧�?始，�? SOF 标记 ----
            // 该标记会�?"本帧第一个入队的像素"写进 FIFO �? tuser 位�??
            if (vsync && !vsync_d)
                frame_sof <= 1'b1;

            //---- �? 3 步：字节相位状�?�机（仅�? href 有效时采样）----
            if (href) begin
                case (byte_phase)
                    S_HIGH : begin
                        // 收到像素�? 1 字节（高字节），锁存并转去等低字�?
                        byte_high_r <= data_in;
                        byte_phase  <= S_LOW;
                    end
                    S_LOW : begin
                        // 收到像素�? 2 字节（低字节），与高字节拼成 16 �? RGB565
                        pixel_rgb565 <= {byte_high_r, data_in};
                        push_pending <= 1'b1;         // 请求下一拍写�? FIFO
                        byte_phase   <= S_HIGH;       // 回到 S_HIGH 等下�?像素
                    end
                    default : begin                    // 防御：非法状态兜�?
                        byte_phase <= S_HIGH;
                    end
                endcase
            end else begin
                // href 无效（行消隐/场消隐）：强制回�? S_HIGH�?
                // 保证每一行的�? 1 个字节永远被当作高字节�??
                byte_phase <= S_HIGH;
            end

            //---- �? 4 步：防御 —�?? 行提前结束且只剩半个像素 ----
            // 正常 RGB565 每行字节数为偶数，不会触发；若出现奇数个字节
            // （时序毛�?/配置错误），把半个像素补零拼合并携带 tlast 输出�?
            // 保住"行结�?"标记，避免下游把两行拼成�?行�??
            if (!href && href_d && (byte_phase == S_LOW) && !push_pending) begin
                pixel_rgb565 <= {byte_high_r, 8'h00};  // 低字节补零强行拼�?
                push_pending <= 1'b1;                  // 入队（tlast 将随它输出）
            end
        end
    end

    //--------------------------------------------------------------------------
    // FWFT 同步 FIFO：first-word-fall-through（首字直通），寄存器输出
    //--------------------------------------------------------------------------
    // 指针宽度 = 地址位宽 + 1 �?"圈数"位，用于区分�?/满：
    //   �? : fifo_wptr == fifo_rptr            （读写指针完全相等）
    //   �? : 低地�?位相�? && 圈数位不�?          （写指针领先读指针整整一圈）
    //
    // FWFT 语义：写入空 FIFO 的数据下�?拍即出现在输出（fifo_dv=1），无需
    // 先发读请求；读操作（tvalid&&tready）每拍弹出一个字并自动预取下�?字�??
    //
    // 读侧握手即背压：tready=0 �? fifo_rd_en=0，读指针冻结，fifo_dout �?
    // fifo_dv 保持不动，AXI 输出完全静止；写侧不受影响，继续入队�?
    // 这正�?"下游没准备好 �? 暂停输出、不丢数�?"的实现点�?
    //
    // 三个核心次�?�公式（均为组合逻辑，在同一拍统�?更新，天然覆�?
    // 读写同时发生、只写�?�只读�?�无操作四种场景）：
    //   写指针次�? : wptr_next = wptr + (写使�? ? 1 : 0)
    //   读指针次�? : rptr_next = rptr + (读使�? ? 1 : 0)
    //   输出有效   : dv_next   = (rptr_next != wptr_next)   �? FIFO 非空即有�?
    //   输出数据   : 若本拍写入的位置正好是下�?读地�? �? 直�?�用写入数据 din
    //                （避免读到存储体旧�?�）；否则预�? mem[rptr_next]�?
    //--------------------------------------------------------------------------
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_wptr <= {(FIFO_AW+1){1'b0}};
            fifo_rptr <= {(FIFO_AW+1){1'b0}};
            fifo_dv   <= 1'b0;
            fifo_dout <= {FIFO_W{1'b0}};
        end else begin
            //---- 写入口：�? din 写入当前写指针指向的存储单元 ----
            if (fifo_wr_en)
                fifo_mem[fifo_wptr[FIFO_AW-1:0]] <= fifo_din;

            //---- 指针推进 ----
            fifo_wptr <= fifo_wptr_next;
            fifo_rptr <= fifo_rptr_next;

            //---- 输出有效：统�?公式（读指针与写指针不重�? = 有数据）----
            fifo_dv <= (fifo_rptr_next != fifo_wptr_next);

            //---- 输出数据：预取下�?读地�?处的�? ----
            if (fifo_wr_en && (fifo_rptr_next[FIFO_AW-1:0] == fifo_wptr[FIFO_AW-1:0]))
                fifo_dout <= fifo_din;   // 本拍写入的字正是下一拍要输出的字 �? 直�??
            else if ((fifo_rptr_next != fifo_wptr_next) && (fifo_rd_en || fifo_wr_en))
                fifo_dout <= fifo_mem[fifo_rptr_next[FIFO_AW-1:0]];  // 预取下一�?
            // 其余情况（无读写活动、或弹空）：fifo_dout 保持原�?�，
            // 保证 AXI 输出数据在暂停期间稳定不变�??
        end
    end

    //====================== AXI4-Stream 输出 ======================
    assign m_axis_tvalid = fifo_dv;             // FIFO 非空即有数据
    assign m_axis_tdata  = fifo_dout[31:0];     // RGB888 像素
    assign m_axis_tlast  = fifo_dout[32];       // �?/帧结束（随最后一个像素）
    assign m_axis_tuser  = fifo_dout[33];       // 帧起�? SOF（随第一个像素）
    assign fifo_usedw    = fifo_wptr - fifo_rptr;  // 占用深度（圈数位参与减法，结果恒正确�?

endmodule
    `ifdef XILINX
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 m_axis_aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF m_axis, ASSOCIATED_RESET rst_n" *)
    `endif
//==============================================================================
// 【综合与集成说明�?
// 1. fifo_mem 用寄存器阵列描述，小深度（≤256）会映射为寄存器/分布�? RAM�?
//    大深度建议换成厂�? FIFO IP（如 Xilinx FIFO Generator，�??
//    "First-Word-Fall-Through" 模式、位�? 26、深�? FIFO_DEPTH），只替换本
//    模块 FIFO �?段，对外 AXI 握手逻辑完全不变，时序更容易收敛�?
// 2. FIFO_DEPTH 必须�? 2 的幂，且不小于最大行宽（像素数）�?
//    默认 2048 适配 1280×720 RGB565�?2592 宽请至少�? 4096�?
// 3. 本模块所有时序�?�辑同步�? pclk；href/vsync/data_in �? OV5640 �? pclk
//    同源驱动，无�?内部同步器（若外部连线噪声明显，可在模块外做两级同步）�??
// 4. 帧的�?后一行：href 先于 vsync 拉低，行尾像素照常携�? tlast，因�?
//    "帧结�?"同样�? tlast 体现；下游按 tlast 统计的行数即有效行数�?
// 5. 若下游不�?�? tuser / fifo_usedw，可悬空；fifo_overflow 建议接到中断�?
//    ILA 触发条件，便于定�? DMA 长时间停摆的问题�?
// 6. 典型对接：m_axis_* �? DMA（如 Xilinx VDMA）的 s_axis_video_*�?
//    tuser �? s_axis_video_tuser[0]（SOF），tlast �? s_axis_video_tlast�?
//==============================================================================


