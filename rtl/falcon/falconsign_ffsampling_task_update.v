`timescale 1ns/1ps
// Module: falconsign_ffsampling_task_update
// Purpose: Falcon ffSampling task scheduler. It walks the dynamic binary
// ffSampling tree and emits coarse split/sample/adjust/merge tasks.
//
// This module turns the recursive ffSampling flow into a stream of 68-bit
// EXU/SamplerZ tasks. It does not perform floating-point arithmetic itself;
// it tracks the logical recursion position, computes SRAM addresses, and emits
// operations in the order required by Falcon:
//   1. sample the right component z1,
//   2. adjust the left component t0 = t0 + (t1 - z1) * l10,
//   3. sample the left component z0,
//   4. merge z0/z1 back toward the root.
//
// bank_q selects the top-level component pass. The first pass samples t1/z1
// (bank_q=1). After the root z1 is available, a special full-size root adjust
// updates t0, then the scheduler restarts from the root for t0/z0 (bank_q=0).
module falconsign_ffsampling_task_update #
(
    parameter LEVEL_W = 4,
    parameter INDEX_W = 10,
    parameter ADDR_W  = 14
)
(
    input                  clk,
    input                  rst_n,

    input                  start,
    output                 start_ready,
    input      [LEVEL_W-1:0] cfg_depth,
    input                  cfg_dynamic_tree,
    input      [ADDR_W-1:0] cfg_t_base,
    input      [ADDR_W-1:0] cfg_tree_base,
    input      [ADDR_W-1:0] cfg_z_base,
    input      [ADDR_W-1:0] cfg_tmp_base,

    output reg             task_valid,
    input                  task_ready,
    output reg [67:0]      task_word,

    input                  task_done,
    input                  task_fail,
    input      [7:0]       task_status,

    output reg             busy,
    output reg             done,
    output reg             fail,
    output reg [7:0]       status,

    output reg [LEVEL_W-1:0] dbg_level,
    output reg [INDEX_W-1:0] dbg_index,
    output reg [1:0]       dbg_state
);

    // Task opcodes consumed by falcon_f64_ffsampling_exu.
    // READ_L10 preloads the tree coefficient stream used by the following
    // SPLIT/ADJUST/MERGE task at the same logical node.
    localparam [3:0] OP_READ_L10     = 4'd0;
    localparam [3:0] OP_SPLIT_T1     = 4'd1;
    localparam [3:0] OP_ADJUST_T0    = 4'd2;
    localparam [3:0] OP_SAMPLE_PAIR  = 4'd3;
    localparam [3:0] OP_MERGE_Z      = 4'd4;
    localparam [3:0] OP_DYNAMIC_LDL  = 4'd5;
    localparam [3:0] OP_COPY         = 4'd6;

    localparam [1:0] ST_RIGHT_DOWN = 2'd0;
    localparam [1:0] ST_RIGHT_UP   = 2'd1;
    localparam [1:0] ST_LEFT_DOWN  = 2'd2;
    localparam [1:0] ST_LEFT_UP    = 2'd3;

    // Per-node micro-steps used before descending into a child.
    // Dynamic-tree mode may generate LDL data first, then every internal node
    // reads l10, splits t1, and preserves the right split half for ADJUST.
    localparam [1:0] SUB_READ     = 2'd0;
    localparam [1:0] SUB_SPLIT    = 2'd1;
    localparam [1:0] SUB_LDL      = 2'd2;
    localparam [1:0] SUB_PRESERVE = 2'd3;

    // RUN_* wraps the valid/ready task handshake:
    // EMIT holds one task stable until accepted; WAIT keeps the scheduler state
    // frozen until EXU reports task_done/task_fail.
    localparam [2:0] RUN_IDLE  = 3'd0;
    localparam [2:0] RUN_EMIT  = 3'd1;
    localparam [2:0] RUN_WAIT  = 3'd2;
    localparam [2:0] RUN_DONE  = 3'd3;
    localparam [2:0] RUN_FAIL  = 3'd4;

    reg [2:0]         run_state;
    reg [LEVEL_W-1:0] level_q;
    reg [INDEX_W-1:0] index_q;
    reg [1:0]         state_q;
    reg [1:0]         sub_q;
    reg               bank_q;
    reg               root_z1_merged_q;
    reg [3:0]         issued_op_q;
    reg [7:0]         fail_status_q;
`ifndef SYNTHESIS
    reg               debug_trace_tasks;
    initial debug_trace_tasks = $test$plusargs("FS_TRACE_TASKS");
`endif

    wire              at_leaf;
    wire              at_root;
    wire              from_right_child;
    wire [LEVEL_W-1:0] child_level;
    wire [INDEX_W-1:0] left_child_index;
    wire [INDEX_W-1:0] right_child_index;
    wire [LEVEL_W-1:0] parent_level;
    wire [INDEX_W-1:0] parent_index;
    wire [ADDR_W-1:0]  src0_addr;
    wire [ADDR_W-1:0]  src1_addr;
    wire [ADDR_W-1:0]  dst_addr;

    // level_q/index_q name the current logical recursion node. Children use
    // binary-heap numbering, which lets index_q[0] tell whether a completed
    // child was the right child when returning to its parent.
    assign start_ready       = (run_state == RUN_IDLE);
    assign at_leaf           = (level_q == cfg_depth);
    assign at_root           = ((level_q == {LEVEL_W{1'b0}}) && (index_q == {INDEX_W{1'b0}}));
    assign from_right_child  = index_q[0];
    assign child_level       = level_q + 1'b1;
    assign left_child_index  = index_q << 1;
    assign right_child_index = (index_q << 1) + 1'b1;
    assign parent_level      = level_q - 1'b1;
    assign parent_index      = index_q >> 1;

    // Traversal state is the hardware version of the recursive call stack:
    //   RIGHT_DOWN: prepare/split a node, then descend into its right child.
    //   RIGHT_UP:   right child returned; emit ADJUST before going left.
    //   LEFT_DOWN:  prepare/split a node, then descend into its left child.
    //   LEFT_UP:    both children returned; emit MERGE and go upward.
    //
    // Data layout:
    // - t/z internal nodes use the compact split layout. A left turn keeps the
    //   same segment offset; a right turn adds that level's child stride.
    // - leaves are addressed by flat coefficient index.
    // - tree data uses Falcon tree buckets per logical recursion level. For an internal node
    // at level d from the root, its L10 polynomial starts at:
    //   d*256 + index*2^(8-d)
    // leaves start at 2304 + index.
    reg [ADDR_W-1:0] component_stride, component_words;
    reg [ADDR_W-1:0] seg_offset, node_words, pair_cnt_for_adj, tree_ofs;
    reg [ADDR_W-1:0] parent_stride;
    reg [ADDR_W-1:0] preserve_offset;
    reg [ADDR_W-1:0] seg_offset_acc;
    reg [INDEX_W-1:0] data_index;
    reg [3:0]         tree_level_eff;
    reg [INDEX_W-1:0] tree_index_eff;
    reg [3:0]         seg_bitpos;
    reg [3:0]         task_opcode_c;
    reg [LEVEL_W-1:0] task_level_c;
    reg [INDEX_W-1:0] task_index_c;
    reg [ADDR_W-1:0]  task_src0_c;
    reg [ADDR_W-1:0]  task_src1_c;
    reg [ADDR_W-1:0]  task_dst_c;
    reg [7:0]         task_aux_c;
    reg [13:0]        task_t0_dst_ext;

    // Number of complex words in one child component below level lv.
    // Ordinary internal ADJUST/COPY/MERGE operations use this half-size count.
    // The root transition from z1 to t0 is different and is marked explicitly
    // by the root_full bit in pack_adjust_task().
    // Tree traversal helpers derived from the current logical node.
    always @(*) begin
        case (cfg_depth - level_q)
            4'd9:  component_stride = {{(ADDR_W-8){1'b0}}, 8'd128};
            4'd8:  component_stride = {{(ADDR_W-7){1'b0}}, 7'd64};
            4'd7:  component_stride = {{(ADDR_W-6){1'b0}}, 6'd32};
            4'd6:  component_stride = {{(ADDR_W-5){1'b0}}, 5'd16};
            4'd5:  component_stride = {{(ADDR_W-4){1'b0}}, 4'd8};
            4'd4:  component_stride = {{(ADDR_W-3){1'b0}}, 3'd4};
            4'd3:  component_stride = {{(ADDR_W-2){1'b0}}, 2'd2};
            4'd2:  component_stride = {{(ADDR_W-1){1'b0}}, 1'd1};
            default: component_stride = {{(ADDR_W-1){1'b0}}, 1'd1};
        endcase
    end
    // Component and subtree sizing derived from cfg_depth/level.
    always @(*) begin
        case (cfg_depth - (level_q - 1'b1))
            4'd9:  parent_stride = {{(ADDR_W-8){1'b0}}, 8'd128};
            4'd8:  parent_stride = {{(ADDR_W-7){1'b0}}, 7'd64};
            4'd7:  parent_stride = {{(ADDR_W-6){1'b0}}, 6'd32};
            4'd6:  parent_stride = {{(ADDR_W-5){1'b0}}, 5'd16};
            4'd5:  parent_stride = {{(ADDR_W-4){1'b0}}, 4'd8};
            4'd4:  parent_stride = {{(ADDR_W-3){1'b0}}, 3'd4};
            4'd3:  parent_stride = {{(ADDR_W-2){1'b0}}, 2'd2};
            4'd2:  parent_stride = {{(ADDR_W-1){1'b0}}, 1'd1};
            default: parent_stride = {{(ADDR_W-1){1'b0}}, 1'd1};
        endcase
    end
    // Segment offset calculation for the current ffSampling node.
    always @(*) begin
        case (level_q)
            4'd0: preserve_offset = {ADDR_W{1'b0}};
            4'd1: preserve_offset = {{(ADDR_W-8){1'b0}}, 8'd128};
            4'd2: preserve_offset = {{(ADDR_W-8){1'b0}}, 8'd192};
            4'd3: preserve_offset = {{(ADDR_W-8){1'b0}}, 8'd224};
            4'd4: preserve_offset = {{(ADDR_W-8){1'b0}}, 8'd240};
            4'd5: preserve_offset = {{(ADDR_W-8){1'b0}}, 8'd248};
            4'd6: preserve_offset = {{(ADDR_W-8){1'b0}}, 8'd252};
            4'd7: preserve_offset = {{(ADDR_W-8){1'b0}}, 8'd254};
            default: preserve_offset = {{(ADDR_W-8){1'b0}}, 8'd255};
        endcase
    end
    // Task word packing: selects opcode, source/destination addresses and aux
    // fields for the next EXU micro-op.
    always @(*) begin
        component_words = {{(ADDR_W-1){1'b0}}, 1'b1} << cfg_depth;
    end

    // Kept as an explicit logical data index for debug/future extensions.
    // Address generation below uses index_q directly.
    always @(*) begin
        if (at_root) begin
            data_index = {INDEX_W{1'b0}};
        end else begin
            data_index = index_q;
        end
    end

    always @(*) begin
        if (at_root) begin
            seg_offset = {ADDR_W{1'b0}};
        end else if (at_leaf) begin
            // Leaves are stored densely as scalar coefficients instead of using
            // the compact internal polynomial segment layout.
            seg_offset = {{(ADDR_W-INDEX_W){1'b0}}, index_q};
        end else begin
            // Internal node offset = sum of child_stride for every right-turn
            // ancestor on the path from the root.
            seg_offset_acc = {ADDR_W{1'b0}};
            for (seg_bitpos = 0; seg_bitpos < 4'd10; seg_bitpos = seg_bitpos + 1) begin
                if ((seg_bitpos < level_q) && index_q[level_q - 1'b1 - seg_bitpos]) begin
                    case (seg_bitpos)
                        4'd0: seg_offset_acc = seg_offset_acc + {{(ADDR_W-8){1'b0}}, 8'd128};
                        4'd1: seg_offset_acc = seg_offset_acc + {{(ADDR_W-7){1'b0}}, 7'd64};
                        4'd2: seg_offset_acc = seg_offset_acc + {{(ADDR_W-6){1'b0}}, 6'd32};
                        4'd3: seg_offset_acc = seg_offset_acc + {{(ADDR_W-5){1'b0}}, 5'd16};
                        4'd4: seg_offset_acc = seg_offset_acc + {{(ADDR_W-4){1'b0}}, 4'd8};
                        4'd5: seg_offset_acc = seg_offset_acc + {{(ADDR_W-3){1'b0}}, 3'd4};
                        4'd6: seg_offset_acc = seg_offset_acc + {{(ADDR_W-2){1'b0}}, 2'd2};
                        default: seg_offset_acc = seg_offset_acc + {{(ADDR_W-1){1'b0}}, 1'd1};
                    endcase
                end
            end
            seg_offset = seg_offset_acc;
        end
    end
    always @(*) begin
        // Number of complex words in one logical component at this node.
        // The main scheduler uses stride_at_level() for child-sized work; this
        // value remains available for sizing/debug around scalar boundaries.
        case (cfg_depth - level_q)
            4'd9:  node_words = {{(ADDR_W-8){1'b0}}, 8'd128};
            4'd8:  node_words = {{(ADDR_W-7){1'b0}}, 7'd64};
            4'd7:  node_words = {{(ADDR_W-6){1'b0}}, 6'd32};
            4'd6:  node_words = {{(ADDR_W-5){1'b0}}, 5'd16};
            4'd5:  node_words = {{(ADDR_W-4){1'b0}}, 4'd8};
            4'd4:  node_words = {{(ADDR_W-3){1'b0}}, 3'd4};
            4'd3:  node_words = {{(ADDR_W-2){1'b0}}, 2'd2};
            4'd2:  node_words = {{(ADDR_W-2){1'b0}}, 2'd2};
            default: node_words = {{(ADDR_W-1){1'b0}}, 1'd1};
        endcase
    end

    always @(*) begin
        pair_cnt_for_adj = component_stride;
    end

    // Target t values live under cfg_t_base; sampled z values live under cfg_z_base.
    // bank_q selects the top-level component:
    //   1: run the whole traversal on t1/z1
    //   0: after root adjust, run the whole traversal on t0/z0
    // split:  t(parent) -> t(left/right)
    // sample: t(leaf), tree(leaf) -> z(leaf)
    // adjust: t(left) += (t(right) - z(right)) * l10
    // merge:  z(left/right) -> z(parent)
    wire [ADDR_W-1:0] t_component_base = cfg_t_base + (bank_q ? component_words : {ADDR_W{1'b0}});
    wire [ADDR_W-1:0] z_component_base = cfg_z_base + (bank_q ? component_words : {ADDR_W{1'b0}});
    // Root SPLIT reads original t from t_area. After that, each child reads
    // from z_area, because its parent SPLIT staged the child input there.
    wire [ADDR_W-1:0] input_component_base = at_root ? t_component_base : z_component_base;
    wire [ADDR_W-1:0] input_addr = input_component_base + seg_offset;
    wire [ADDR_W-1:0] t_addr    = t_component_base + seg_offset;
    wire [ADDR_W-1:0] z_addr    = z_component_base + seg_offset;
    wire              at_scalar_split = (level_q == (cfg_depth - 1'b1));
    wire [INDEX_W-1:0] local_index = index_q;
    wire [INDEX_W-1:0] local_leaf_index = index_q;
    // The last internal split produces a scalar pair. Keep the two scalar
    // inputs in z_area and write sampled leaf outputs into tmp_leaf_base so the
    // later scalar ADJUST can still read the original t1 scalar.
    wire [ADDR_W-1:0] scalar_pair_addr = z_component_base + {{(ADDR_W-INDEX_W){1'b0}}, (local_index << 1)};
    wire [ADDR_W-1:0] scalar_leaf_addr = z_component_base + {{(ADDR_W-INDEX_W){1'b0}}, local_leaf_index};
    wire [ADDR_W-1:0] tmp_addr = cfg_tmp_base + seg_offset;
    // Scalar samples need 512 words and must not overlap the compact
    // internal merge scratch at cfg_tmp_base..cfg_tmp_base+255.
    wire [ADDR_W-1:0] tmp_leaf_base = cfg_tmp_base - component_words - {{(ADDR_W-8){1'b0}}, 8'd128};
    wire [ADDR_W-1:0] tmp_leaf_addr = tmp_leaf_base + {{(ADDR_W-INDEX_W){1'b0}}, local_leaf_index};
    wire [ADDR_W-1:0] tmp_scalar_pair_addr = tmp_leaf_base + {{(ADDR_W-INDEX_W){1'b0}}, (local_index << 1)};
    wire [ADDR_W-1:0] preserve_base = cfg_tmp_base + {{(ADDR_W-9){1'b0}}, 9'd256};
    wire [ADDR_W-1:0] preserve_addr = preserve_base + preserve_offset;
    // After the complete z1 tree has been merged at the root, emit one
    // full-size root ADJUST to update t0 before starting the z0 pass.
    wire              outer_root_adjust = at_root && bank_q && root_z1_merged_q;

    always @(*) begin
        // Map traversal position to the Falcon tree entry consumed by the next
        // task. Ordinary child work uses level+1; scalar-pair work at level 8
        // shares a tree entry for two leaves; outer_root_adjust uses root l10.
        if (outer_root_adjust) begin
            tree_level_eff = 4'd0;
            tree_index_eff = {INDEX_W{1'b0}};
        end else if (at_leaf) begin
            tree_level_eff = 4'd9;
            tree_index_eff = (bank_q ? {{(INDEX_W-9){1'b0}}, 9'd256} : {INDEX_W{1'b0}})
                           + index_q[7:0];
        end else if (level_q == 4'd8) begin
            tree_level_eff = 4'd8;
            tree_index_eff = (bank_q ? {{(INDEX_W-8){1'b0}}, 8'd128} : {INDEX_W{1'b0}})
                           + (index_q >> 1);
        end else begin
            tree_level_eff = level_q + 1'b1;
            tree_index_eff = (bank_q ? ({{(INDEX_W-1){1'b0}}, 1'b1} << level_q) : {INDEX_W{1'b0}})
                           + index_q;
        end
    end

    always @(*) begin
        // Falcon tree memory is laid out as fixed buckets:
        //   level 0 : 1 polynomial  * 256 words
        //   level 1 : 2 polynomials * 128 words
        //   ...
        //   level 8 : 256 scalar-pair entries
        //   level 9 : 512 leaf sigma entries
        case (tree_level_eff)
            4'd0: tree_ofs = {ADDR_W{1'b0}} + (tree_index_eff << 8);
            4'd1: tree_ofs = {{(ADDR_W-9){1'b0}}, 9'd256} + (tree_index_eff << 7);
            4'd2: tree_ofs = {{(ADDR_W-10){1'b0}}, 10'd512} + (tree_index_eff << 6);
            4'd3: tree_ofs = {{(ADDR_W-10){1'b0}}, 10'd768} + (tree_index_eff << 5);
            4'd4: tree_ofs = {{(ADDR_W-11){1'b0}}, 11'd1024} + (tree_index_eff << 4);
            4'd5: tree_ofs = {{(ADDR_W-11){1'b0}}, 11'd1280} + (tree_index_eff << 3);
            4'd6: tree_ofs = {{(ADDR_W-11){1'b0}}, 11'd1536} + (tree_index_eff << 2);
            4'd7: tree_ofs = {{(ADDR_W-11){1'b0}}, 11'd1792} + (tree_index_eff << 1);
            4'd8: tree_ofs = {{(ADDR_W-12){1'b0}}, 12'd2048} + tree_index_eff;
            default: tree_ofs = {{(ADDR_W-12){1'b0}}, 12'd2304} + tree_index_eff;
        endcase
    end

    wire [ADDR_W-1:0] tree_addr = cfg_tree_base + tree_ofs;

    // SPLIT consumes the current node input and l10. For the scalar-pair split,
    // the output is redirected to flat scalar storage instead of compact z_addr.
    wire [ADDR_W-1:0] split_src0 = input_addr;
    wire [ADDR_W-1:0] split_src1 = tree_addr;
    wire [ADDR_W-1:0] split_dst  = at_scalar_split ? scalar_pair_addr : z_addr;
    wire [ADDR_W-1:0] preserve_src = at_scalar_split ? (scalar_pair_addr + 1'b1)
                                                     : (z_addr + pair_cnt_for_adj);

    // SAMPLE consumes one scalar t value and its sigma entry. Internal nodes do
    // not sample directly, but these aliases keep dynamic-tree task packing
    // using the same task format.
    wire [ADDR_W-1:0] sample_src0 = at_leaf ? scalar_leaf_addr : input_addr;
    wire [ADDR_W-1:0] sample_src1 = tree_addr;
    // Scalar leaf SAMPLE: write z to TMP (preserves t0/t1 at scalar_pair_addr for ADJUST).
    wire [ADDR_W-1:0] sample_dst  = at_leaf ? tmp_leaf_addr : z_addr;

    // Child MERGE writes to TMP instead of z_area, preserving SPLIT t1 for
    // ADJUST. A child node's own compact tmp_addr already includes its path
    // offset, so a right child lands at parent_tmp + parent_stride.
    wire [ADDR_W-1:0] merge_dst_child = tmp_addr;
    wire [ADDR_W-1:0] merge_dst  = at_root ? z_addr : merge_dst_child;
    // Parent MERGE reads both children's z from TMP.
    wire [ADDR_W-1:0] merge_src0 = at_scalar_split ? tmp_scalar_pair_addr : tmp_addr;
    wire [ADDR_W-1:0] merge_src1 = tree_addr;

    // ADJUST source convention:
    // - inner ADJUST reads preserved split-right t1 and the right-child z from
    //   TMP, then updates the left input staged in z_area/scalar_pair_addr.
    // - outer_root_adjust reads full root t1/z1 regions and writes back to t0.
    wire [ADDR_W-1:0] adj_src0 = outer_root_adjust ? (cfg_t_base + component_words)
                                                   : (at_scalar_split ? (scalar_pair_addr + 1'b1)
                                                                      : preserve_addr);
    wire [ADDR_W-1:0] adj_src1 = outer_root_adjust ? (cfg_z_base + component_words)
                                                   : (at_scalar_split ? (tmp_scalar_pair_addr + 1'b1)
                                                                      : (tmp_addr + pair_cnt_for_adj));
    wire [ADDR_W-1:0] adj_dst  = outer_root_adjust ? cfg_t_base
                                                   : (at_scalar_split ? scalar_pair_addr : z_addr);

    assign src0_addr = sample_src0;
    assign src1_addr = sample_src1;
    assign dst_addr  = sample_dst;

    always @(*) begin
        task_valid = 1'b0;
        task_word  = 68'd0;
        task_opcode_c = OP_SAMPLE_PAIR;
        task_level_c  = level_q;
        task_index_c  = index_q;
        task_src0_c   = sample_src0;
        task_src1_c   = sample_src1;
        task_dst_c    = sample_dst;
        task_aux_c    = 8'hFF;
        task_t0_dst_ext = 14'd0;

        if (run_state == RUN_EMIT) begin
            task_valid = 1'b1;
            case (state_q)
                ST_RIGHT_DOWN: begin
                    // Descending into a right branch: split this node if
                    // internal, otherwise sample the leaf.
                    if (at_leaf) begin
                        task_opcode_c = OP_SAMPLE_PAIR;
                        task_aux_c    = 8'h00;
                    end else if (cfg_dynamic_tree && (sub_q == SUB_LDL)) begin
                        task_opcode_c = OP_DYNAMIC_LDL;
                        task_src0_c   = src0_addr;
                        task_src1_c   = src1_addr;
                        task_dst_c    = dst_addr;
                        task_aux_c    = 8'h00;
                    end else if (sub_q == SUB_PRESERVE) begin
                        task_opcode_c = OP_COPY;
                        task_src0_c   = preserve_src;
                        task_src1_c   = tree_addr;
                        task_dst_c    = preserve_addr;
                        task_aux_c    = 8'h00;
                    end else begin
                        task_opcode_c = OP_SPLIT_T1;
                        task_src0_c   = split_src0;
                        task_src1_c   = split_src1;
                        task_dst_c    = split_dst;
                        task_aux_c    = 8'h00;
                    end
                end

                ST_RIGHT_UP: begin
                    // Right subtree is complete, so compute the Falcon adjust
                    // before scheduling the left subtree.
                    task_t0_dst_ext = {{(14-ADDR_W){1'b0}}, adj_dst};
                    task_opcode_c = OP_ADJUST_T0;
                    task_level_c  = level_q;
                    task_index_c  = adj_dst[9:0];
                    task_src0_c   = adj_src0;
                    task_src1_c   = tree_addr;
                    task_dst_c    = adj_src1;
                    task_aux_c    = {task_t0_dst_ext[13:10], 3'd0, outer_root_adjust};
`ifndef SYNTHESIS
                    if (debug_trace_tasks && outer_root_adjust) begin
                        $display("  FS_OUTER_ROOT_ADJ bank=%0d root_z1=%0d: adj_dst=%0d adj_src0=%0d adj_src1=%0d",
                                 bank_q, root_z1_merged_q, adj_dst, adj_src0, adj_src1);
                    end else if (debug_trace_tasks) begin
                        $display("  FS_INNER_ADJ L=%0d I=%0d bank=%0d: adj_dst=%0d adj_src0=%0d adj_src1=%0d",
                                 level_q, index_q, bank_q, adj_dst, adj_src0, adj_src1);
                    end
`endif
                end

                ST_LEFT_DOWN: begin
                    // Same preparation steps as RIGHT_DOWN, but the next
                    // descent target is the left child in the FSM update.
                    if (at_leaf) begin
                        task_opcode_c = OP_SAMPLE_PAIR;
                        task_aux_c    = 8'h00;
                    end else if (cfg_dynamic_tree && (sub_q == SUB_LDL)) begin
                        task_opcode_c = OP_DYNAMIC_LDL;
                        task_src0_c   = src0_addr;
                        task_src1_c   = src1_addr;
                        task_dst_c    = dst_addr;
                        task_aux_c    = 8'h00;
                    end else if (sub_q == SUB_PRESERVE) begin
                        task_opcode_c = OP_COPY;
                        task_src0_c   = preserve_src;
                        task_src1_c   = tree_addr;
                        task_dst_c    = preserve_addr;
                        task_aux_c    = 8'h00;
                    end else begin
                        task_opcode_c = OP_SPLIT_T1;
                        task_src0_c   = split_src0;
                        task_src1_c   = split_src1;
                        task_dst_c    = split_dst;
                        task_aux_c    = 8'h00;
                    end
                end

                ST_LEFT_UP: begin
                    // Both children under this node are ready; merge z0/z1
                    // toward the parent.
                    task_opcode_c = OP_MERGE_Z;
                    task_src0_c   = merge_src0;
                    task_src1_c   = merge_src1;
                    task_dst_c    = merge_dst;
                    task_aux_c    = 8'h00;
                end

                default: begin
                    task_opcode_c = OP_SAMPLE_PAIR;
                    task_aux_c    = 8'hFF;
                end
            endcase
        end

        // Generic task word format:
        //   [67:64] opcode, [63:60] level, [59:50] index,
        //   [49:36] src0, [35:22] src1, [21:8] dst, [7:0] aux.
        task_word[67:64] = task_opcode_c;
        task_word[63:60] = task_level_c[3:0];
        task_word[59:50] = task_index_c[9:0];
        task_word[49:36] = {{(14-ADDR_W){1'b0}}, task_src0_c};
        task_word[35:22] = {{(14-ADDR_W){1'b0}}, task_src1_c};
        task_word[21:8]  = {{(14-ADDR_W){1'b0}}, task_dst_c};
        task_word[7:0]   = task_aux_c;
    end

    // Non-recursive ffSampling scheduler FSM. It walks left/right children,
    // emits stable valid/ready task words, and advances after task_done.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_state     <= RUN_IDLE;
            level_q       <= {LEVEL_W{1'b0}};
            index_q       <= {INDEX_W{1'b0}};
            state_q       <= ST_RIGHT_DOWN;
            sub_q         <= SUB_SPLIT;
            bank_q        <= 1'b1;
            root_z1_merged_q <= 1'b0;
            issued_op_q   <= 4'd0;
            fail_status_q <= 8'h00;
            busy          <= 1'b0;
            done          <= 1'b0;
            fail          <= 1'b0;
            status        <= 8'h00;
            dbg_level     <= {LEVEL_W{1'b0}};
            dbg_index     <= {INDEX_W{1'b0}};
            dbg_state     <= ST_RIGHT_DOWN;
        end else begin
            done      <= 1'b0;
            fail      <= 1'b0;
            dbg_level <= level_q;
            dbg_index <= index_q;
            dbg_state <= state_q;

            case (run_state)
                RUN_IDLE: begin
                    busy   <= 1'b0;
                    status <= 8'h00;
                    if (start) begin
                        // cfg_depth is the Falcon recursion depth. Depth 9 is
                        // the normal Falcon-512 path; the guard catches invalid
                        // bring-up configurations before any task is emitted.
                        if ((cfg_depth == {LEVEL_W{1'b0}}) || (cfg_depth > 4'd10)) begin
                            fail_status_q <= 8'hE1;
                            busy          <= 1'b1;
                            run_state     <= RUN_FAIL;
                        end else begin
                            // Start every signature at the root of the t1/z1
                            // pass. Dynamic-tree mode inserts LDL before the
                                // normal READ/SPLIT steps at each internal node.
                            level_q   <= {LEVEL_W{1'b0}};
                            index_q   <= {INDEX_W{1'b0}};
                            state_q   <= ST_RIGHT_DOWN;
                            sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_SPLIT;
                            bank_q    <= 1'b1;
                            root_z1_merged_q <= 1'b0;
                            busy      <= 1'b1;
                            run_state <= RUN_EMIT;
                        end
                    end
                end

                RUN_EMIT: begin
                    if (task_valid && task_ready) begin
                        // Remember the accepted opcode so the WAIT state can
                        // advance the traversal when the EXU completes.
                        issued_op_q <= task_word[67:64];
                        run_state   <= RUN_WAIT;
                    end
                end

                RUN_WAIT: begin
                    if (task_done) begin
                        if (task_fail) begin
                            // Propagate EXU/SamplerZ failure status unchanged;
                            // top-level control can decide whether to restart.
                            fail_status_q <= task_status;
                            run_state     <= RUN_FAIL;
                        end else begin
                            if (issued_op_q == OP_DYNAMIC_LDL) begin
                                // Dynamic LDL only prepares tree data for this
                                // node; continue with SPLIT (tree coefficient
                                // is read by ADJUST, not needed for SPLIT).
                                sub_q     <= SUB_SPLIT;
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_SPLIT_T1) begin
                                if (!at_scalar_split) begin
                                    // Internal split: save the right split half
                                    // before descending, because right sampling
                                    // uses the same z/TMP scratch space.
                                    sub_q     <= SUB_PRESERVE;
                                    run_state <= RUN_EMIT;
                                end else begin
                                    // Scalar split already left both scalar
                                    // inputs in z_area; descend directly to the
                                    // right scalar leaf.
                                    level_q   <= child_level;
                                    index_q   <= right_child_index;
                                    state_q   <= ST_RIGHT_DOWN;
                                    sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_SPLIT;
                                    run_state <= RUN_EMIT;
                                end
                            end else if (issued_op_q == OP_COPY) begin
                                // The split-right half is preserved; descend
                                // into the right child first, matching Falcon's
                                // recursive order.
                                level_q   <= child_level;
                                index_q   <= right_child_index;
                                state_q   <= ST_RIGHT_DOWN;
                                sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_SPLIT;
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_ADJUST_T0) begin
                                if (outer_root_adjust) begin
                                    // z1 is complete and t0 has just been root
                                    // adjusted. Restart the traversal for the
                                    // second top-level component, z0.
                                    bank_q    <= 1'b0;
                                    level_q   <= {LEVEL_W{1'b0}};
                                    index_q   <= {INDEX_W{1'b0}};
                                    state_q   <= ST_RIGHT_DOWN;
                                    sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_SPLIT;
                                    root_z1_merged_q <= 1'b0;
                                end else begin
                                    // Ordinary adjust completed after the right
                                    // subtree, so now sample the left subtree.
                                    level_q   <= child_level;
                                    index_q   <= left_child_index;
                                    state_q   <= ST_LEFT_DOWN;
                                    sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_SPLIT;
                                end
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_SAMPLE_PAIR) begin
                                // A leaf returns to its parent. Right leaves
                                // trigger ADJUST; left leaves trigger MERGE.
                                level_q   <= parent_level;
                                index_q   <= parent_index;
                                state_q   <= from_right_child ? ST_RIGHT_UP : ST_LEFT_UP;
                                sub_q     <= SUB_READ;
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_MERGE_Z) begin
                                if (at_root) begin
                                    if (bank_q && !root_z1_merged_q) begin
                                        // First root merge produced full z1.
                                        // Emit the special root ADJUST next.
                                        root_z1_merged_q <= 1'b1;
                                        state_q          <= ST_RIGHT_UP;
                                        sub_q            <= SUB_READ;
                                        run_state        <= RUN_EMIT;
                                    end else begin
                                        // Second root merge produced full z0;
                                        // ffSampling is complete.
                                        run_state <= RUN_DONE;
                                    end
                                end else begin
                                    // Non-root merge returns to the parent.
                                    level_q   <= parent_level;
                                    index_q   <= parent_index;
                                    state_q <= from_right_child ? ST_RIGHT_UP : ST_LEFT_UP;
                                    sub_q     <= SUB_READ;
                                    run_state <= RUN_EMIT;
                                end
                            end else begin
                                fail_status_q <= 8'hE2;
                                run_state     <= RUN_FAIL;
                            end
                        end
                    end
                end

                RUN_DONE: begin
                    busy      <= 1'b0;
                    done      <= 1'b1;
                    status    <= 8'h00;
                    run_state <= RUN_IDLE;
                end

                RUN_FAIL: begin
                    busy      <= 1'b0;
                    fail      <= 1'b1;
                    status    <= fail_status_q;
                    run_state <= RUN_IDLE;
                end

                default: begin
                    busy      <= 1'b0;
                    run_state <= RUN_IDLE;
                end
            endcase
        end
    end

endmodule
