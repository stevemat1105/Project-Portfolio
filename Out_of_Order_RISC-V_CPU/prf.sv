// Physical Register File (PRF) - Step 3: Array-Based Exec Ports
//
// Merged reg file (ERR Design)
// - stores both speculative and committed values
// - 64 physical regs: p0-p31 (architectural), p32-p63 (speculative)
// - p0 is hardwired to 0
// - rename read ports: 4 ports (2 per instruction for 2-way superscalar)
// - exec read ports: NUM_EXEC_READ_PORTS ports
// - MULTI port write from CDB array
// - internal forwarding to handle RAW hazards from all cdb ports

module prf
import types::*;
#(
    parameter NUM_PREGS = 64,
    parameter NUM_EXEC_READ_PORTS = 7,
    parameter NUM_RENAME_READ_PORTS = 4
)(
    input  logic        clk,
    input  logic        rst,

     
    // RENAME READ PORTS - Instruction 0
     
    input  logic [5:0]  rename_rs1_preg,
    output logic [31:0] rename_rs1_data,

    input  logic [5:0]  rename_rs2_preg,
    output logic [31:0] rename_rs2_data,

     
    // RENAME READ PORTS - Instruction 1 (superscalar)
     
    input  logic [5:0]  rename_rs1_preg_1,
    output logic [31:0] rename_rs1_data_1,

    input  logic [5:0]  rename_rs2_preg_1,
    output logic [31:0] rename_rs2_data_1,

     
    // EXEC READ PORTS
     
    // [port_idx][operand_idx where 0=rs1, 1=rs2]
    input  logic [5:0]  exec_rs_preg [NUM_EXEC_READ_PORTS][2],
    output logic [31:0] exec_rs_data [NUM_EXEC_READ_PORTS][2],

    // write port from CDB
    input  cdb_t        cdb [NUM_CDB_PORTS]
);

    logic [31:0] registers [NUM_PREGS];
    
    always_comb begin
         
        // INSTRUCTION 0 RENAME READ PORTS
         

        // RENAME_RS1 forwarding (inst 0)
        if (rename_rs1_preg == 6'd0) begin
            rename_rs1_data = 32'h0;
        end
        else begin
            rename_rs1_data = registers[rename_rs1_preg];
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid && cdb[i].preg == rename_rs1_preg)
                    rename_rs1_data = cdb[i].data;
            end
        end

        // RENAME_RS2 forwarding (inst 0)
        if (rename_rs2_preg == 6'd0) begin
            rename_rs2_data = 32'h0;
        end
        else begin
            rename_rs2_data = registers[rename_rs2_preg];
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid && cdb[i].preg == rename_rs2_preg)
                    rename_rs2_data = cdb[i].data;
            end
        end

         
        // INSTRUCTION 1 RENAME READ PORTS (superscalar)
         

        // RENAME_RS1 forwarding (inst 1)
        if (rename_rs1_preg_1 == 6'd0) begin
            rename_rs1_data_1 = 32'h0;
        end
        else begin
            rename_rs1_data_1 = registers[rename_rs1_preg_1];
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid && cdb[i].preg == rename_rs1_preg_1)
                    rename_rs1_data_1 = cdb[i].data;
            end
        end

        // RENAME_RS2 forwarding (inst 1)
        if (rename_rs2_preg_1 == 6'd0) begin
            rename_rs2_data_1 = 32'h0;
        end
        else begin
            rename_rs2_data_1 = registers[rename_rs2_preg_1];
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid && cdb[i].preg == rename_rs2_preg_1)
                    rename_rs2_data_1 = cdb[i].data;
            end
        end

         
        // EXEC READ PORTS
         
        for (integer port = 0; port < NUM_EXEC_READ_PORTS; port++) begin
            for (integer operand = 0; operand < 2; operand++) begin
                if (exec_rs_preg[port][operand] == 6'd0) begin
                    exec_rs_data[port][operand] = 32'h0;
                end 
                else begin
                    exec_rs_data[port][operand] = registers[exec_rs_preg[port][operand]];
                    for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                        if (cdb[i].valid && cdb[i].preg == exec_rs_preg[port][operand])
                            exec_rs_data[port][operand] = cdb[i].data;
                    end
                end
            end
        end
    end
    
    // write from all valid CDB ports
    always_ff @(posedge clk) begin
        if (rst) begin
            // initialize all regs to 0
            for (integer i = 0; i < NUM_PREGS; i++) begin
                registers[i] <= 32'h0;
            end
        end 
        else begin
            // write from all valid CDB ports
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid && cdb[i].preg != 6'd0)
                    registers[cdb[i].preg] <= cdb[i].data;
            end
        end
    end
endmodule : prf