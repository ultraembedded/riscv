//-----------------------------------------------------------------
// Module
//-----------------------------------------------------------------
module inst_gen
(
    input               clk_i,
    input               rst_i,

    // Fetched opcode
    output reg [31:0]   opcode_o,
    output reg [31:0]   opcode_pc_o,
    output              opcode_valid_o,

    // Branch target
    input               branch_i,
    input [31:0]        branch_pc_i,
    input               stall_i,

    // Decoded register details
    output [4:0]        rs1_o,
    output [4:0]        rs2_o,
    output [4:0]        rd_o
);

//-----------------------------------------------------------------
// Random Instruction Generator
//-----------------------------------------------------------------
reg opcode_valid_q;

always @(posedge rst_i or posedge clk_i)
if (rst_i)
begin
    opcode_o <= 32'bz;
    opcode_valid_q <= 1'b0;
end
else
begin
    reg [31:0] instruction;
    instruction = 32'bz;

    case ($urandom_range(48, 0))
    0: // add
    begin
        instruction = 'h33;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    1: // addi
    begin
        instruction = 'h13;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    2: // and
    begin
        instruction = 'h7033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    3: // andi
    begin
        instruction = 'h7013;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    4: // auipc
    begin
        instruction = 'h17;
        instruction[11:7] = $urandom; // rd
        instruction[31:12] = $urandom; // imm20
    end
    5: // beq
    begin
        instruction = 'h63;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31] = $urandom; // bimm
        instruction[7] = $urandom; // bimm
        instruction[30:25] = $urandom; // bimm
        instruction[11:8] = $urandom; // bimm
    end
    6: // bge
    begin
        instruction = 'h5063;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31] = $urandom; // bimm
        instruction[7] = $urandom; // bimm
        instruction[30:25] = $urandom; // bimm
        instruction[11:8] = $urandom; // bimm
    end
    7: // bgeu
    begin
        instruction = 'h7063;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31] = $urandom; // bimm
        instruction[7] = $urandom; // bimm
        instruction[30:25] = $urandom; // bimm
        instruction[11:8] = $urandom; // bimm
    end
    8: // blt
    begin
        instruction = 'h4063;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31] = $urandom; // bimm
        instruction[7] = $urandom; // bimm
        instruction[30:25] = $urandom; // bimm
        instruction[11:8] = $urandom; // bimm
    end
    9: // bltu
    begin
        instruction = 'h6063;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31] = $urandom; // bimm
        instruction[7] = $urandom; // bimm
        instruction[30:25] = $urandom; // bimm
        instruction[11:8] = $urandom; // bimm
    end
    10: // bne
    begin
        instruction = 'h1063;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31] = $urandom; // bimm
        instruction[7] = $urandom; // bimm
        instruction[30:25] = $urandom; // bimm
        instruction[11:8] = $urandom; // bimm
    end
    11: // csrrc
    begin
        instruction = 'h3073;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    12: // csrrci
    begin
        instruction = 'h7073;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    13: // csrrs
    begin
        instruction = 'h2073;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    14: // csrrsi
    begin
        instruction = 'h6073;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    15: // csrrw
    begin
        instruction = 'h1073;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    16: // csrrwi
    begin
        instruction = 'h5073;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    17: // jal
    begin
        instruction = 'h6f;
        instruction[11:7] = $urandom; // rd
        instruction[19:12] = $urandom; // jimm20
        instruction[20] = $urandom; // jimm20
        instruction[30:25] = $urandom; // jimm20
        instruction[24:21] = $urandom; // jimm20
    end
    18: // jalr
    begin
        instruction = 'h67;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    19: // lb
    begin
        instruction = 'h3;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    20: // lbu
    begin
        instruction = 'h4003;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    21: // lh
    begin
        instruction = 'h1003;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    22: // lhu
    begin
        instruction = 'h5003;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    23: // lui
    begin
        instruction = 'h37;
        instruction[11:7] = $urandom; // rd
        instruction[31:12] = $urandom; // imm20
    end
    24: // lw
    begin
        instruction = 'h2003;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    25: // lwu
    begin
        instruction = 'h6003;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    26: // or
    begin
        instruction = 'h6033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    27: // ori
    begin
        instruction = 'h6013;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    28: // sb
    begin
        instruction = 'h23;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31:25] = $urandom; // storeimm
        instruction[11:7] = $urandom; // storeimm
    end
    29: // sbreak
    begin
        instruction = 'h100073;
    end
    30: // scall
    begin
        instruction = 'h73;
    end
    31: // sh
    begin
        instruction = 'h1023;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31:25] = $urandom; // storeimm
        instruction[11:7] = $urandom; // storeimm
    end
    32: // sll
    begin
        instruction = 'h1033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    33: // slli
    begin
        instruction = 'h1013;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // shamt
    end
    34: // slt
    begin
        instruction = 'h2033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    35: // slti
    begin
        instruction = 'h2013;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    36: // sltiu
    begin
        instruction = 'h3013;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    37: // sltu
    begin
        instruction = 'h3033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    38: // sra
    begin
        instruction = 'h40005033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    39: // srai
    begin
        instruction = 'h40005013;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // shamt
    end
    40: // sret
    begin
        instruction = 'h80000073;
    end
    41: // srl
    begin
        instruction = 'h5033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    42: // srli
    begin
        instruction = 'h5013;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // shamt
    end
    43: // sub
    begin
        instruction = 'h40000033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    44: // sw
    begin
        instruction = 'h2023;
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
        instruction[31:25] = $urandom; // storeimm
        instruction[11:7] = $urandom; // storeimm
    end
    45: // xor
    begin
        instruction = 'h4033;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[24:20] = $urandom; // rs2
    end
    46: // xori
    begin
        instruction = 'h4013;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = $urandom; // imm12
    end
    // Special cases
    47: // csrrsi SR
    begin
        instruction = 'h6073;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = 12'h50a; // imm12
    end
    48: // csrrci SR
    begin
        instruction = 'h7073;
        instruction[11:7] = $urandom; // rd
        instruction[19:15] = $urandom; // rs1
        instruction[31:20] = 12'h50a; // imm12
    end    
    endcase

    if (stall_i)
        ;
    else if ($urandom_range(8, 0) == 0)
    begin
        opcode_o       <= 32'bz;
        opcode_valid_q <= 1'b0;
    end
    else
    begin
        opcode_o        <= instruction;
        opcode_valid_q  <= 1'b1;
    end
end

assign opcode_valid_o = opcode_valid_q && !branch_i;

//-----------------------------------------------------------------
// Fetch Checker
//-----------------------------------------------------------------
reg [31:0] next_pc;
initial
begin
    next_pc = 'h100;
    forever
    begin
        @(posedge clk_i);

        if (opcode_valid_o && !stall_i)
            next_pc = next_pc + 4;

        if (branch_i)
            next_pc = branch_pc_i;
    end
end


assign opcode_pc_o = next_pc;

//-------------------------------------------------------------------
// Opcode output
//-------------------------------------------------------------------
assign rs1_o           = opcode_o[19:15];
assign rs2_o           = opcode_o[24:20];
assign rd_o            = opcode_o[11:7];

endmodule