// PRF Read Port Arbiter
// arbitrate access to shared exec read ports
// Priority: Array index 0 = highest priority

module prf_read_arbiter
import types::*;
#(
    parameter NUM_EXEC_UNITS = 3,           // no. of requesters
    parameter NUM_EXEC_READ_PORTS = 2       // no. of available PRF exec read ports
)(
    // requests from exec units
    input  logic                    exec_req [NUM_EXEC_UNITS],
    input  logic [5:0]              exec_rs1_preg [NUM_EXEC_UNITS],
    input  logic [5:0]              exec_rs2_preg [NUM_EXEC_UNITS],
    
    // grants back to exec units
    output logic                    exec_grant [NUM_EXEC_UNITS],
    
    // to PRF exec ports (indexed by port, operand)
    output logic [5:0]              prf_exec_preg [NUM_EXEC_READ_PORTS][2],     // [port][0=rs1, 1=rs2]
    
    // from PRF exec ports (indexed by port, then operand)
    input  logic [31:0]             prf_exec_data [NUM_EXEC_READ_PORTS][2],     // [port][0=rs1, 1=rs2]
    
    // wack to exec units (indexed by unit, then operand)
    output logic [31:0]             exec_rs1_data [NUM_EXEC_UNITS],
    output logic [31:0]             exec_rs2_data [NUM_EXEC_UNITS]
);

generate
    if (NUM_EXEC_UNITS == NUM_EXEC_READ_PORTS) begin
        for (genvar unit = 0; unit < NUM_EXEC_UNITS; unit++) begin
            assign exec_grant[unit] = exec_req[unit];

            assign prf_exec_preg[unit][0] = exec_rs1_preg[unit];
            assign prf_exec_preg[unit][1] = exec_rs2_preg[unit];

            assign exec_rs1_data[unit] = prf_exec_data[unit][0];
            assign exec_rs2_data[unit] = prf_exec_data[unit][1];
        end

    end else begin

        always_comb begin
            automatic integer port_idx;

            // initialize all grants to 0
            for (integer unit = 0; unit < NUM_EXEC_UNITS; unit++) begin
                exec_grant[unit] = 1'b0;
            end

            // initialize all PRF port requests to 0
            for (integer port = 0; port < NUM_EXEC_READ_PORTS; port++) begin
                prf_exec_preg[port][0] = 6'b0;      // rs1
                prf_exec_preg[port][1] = 6'b0;      // rs2
            end

            port_idx = 0;

            for (integer unit = 0; unit < NUM_EXEC_UNITS && port_idx < NUM_EXEC_READ_PORTS; unit++) begin
                if (exec_req[unit]) begin
                    // grant this unit access to a PRF port
                    exec_grant[unit] = 1'b1;

                    // assign this unit's operands to the next available port
                    prf_exec_preg[port_idx][0] = exec_rs1_preg[unit];
                    prf_exec_preg[port_idx][1] = exec_rs2_preg[unit];

                    port_idx++;
                end
            end
        end

        // route data from PRF ports back to the granted exec units
        always_comb begin
            automatic integer port_idx;

            // initialize all exec unit data outputs to 0
            for (integer unit = 0; unit < NUM_EXEC_UNITS; unit++) begin
                exec_rs1_data[unit] = 32'h0;
                exec_rs2_data[unit] = 32'h0;
            end

            // route data back: match granted units to PRF ports
            port_idx = 0;

            for (integer unit = 0; unit < NUM_EXEC_UNITS && port_idx < NUM_EXEC_READ_PORTS; unit++) begin
                if (exec_grant[unit]) begin
                    // route data from this port to this unit
                    exec_rs1_data[unit] = prf_exec_data[port_idx][0];
                    exec_rs2_data[unit] = prf_exec_data[port_idx][1];

                    port_idx++;
                end
            end
        end
    end
endgenerate
endmodule