// N-WAY CDB ARBITER
// multiple exec units to broadcast simultaneously
//
// Priority: array order determines priority (exec_cdb_req[0] = highest priority)

module cdb_arbiter
    import types::*;
#(
    parameter NUM_CDB_PORTS = 2,    // no. of simultaneous broadcasts
    parameter NUM_EXEC_UNITS = 3    // no. of exec units
)(
    // request/grant intf with exec units
    input  logic        exec_cdb_req [NUM_EXEC_UNITS],      // request CDB access
    input  cdb_t        exec_cdb_data [NUM_EXEC_UNITS],     // data to broadcast
    output logic        exec_cdb_grant [NUM_EXEC_UNITS],    // grant CDB access
    
    // to CDB
    output cdb_t        cdb_out [NUM_CDB_PORTS]
);

generate
    if (NUM_EXEC_UNITS == NUM_CDB_PORTS) begin
        for (genvar i = 0; i < NUM_EXEC_UNITS; i++) begin
            assign exec_cdb_grant[i] = exec_cdb_req[i];
            assign cdb_out[i] = exec_cdb_req[i] ? exec_cdb_data[i] : '0;
        end

    end else begin
        // priority: lower index = higher priority

        always_comb begin
            integer cdb_port;
            cdb_port = 0;

            // initialize all CDB outputs to invalid
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                cdb_out[i] = '0;
            end

            for (integer i = 0; i < NUM_EXEC_UNITS; i++) begin
                exec_cdb_grant[i] = 1'b0;
            end

            // grant CDB ports in priority order
            // only grant if exec unit is requesting and we have available ports
            for (integer i = 0; i < NUM_EXEC_UNITS; i++) begin
                if (exec_cdb_req[i] && cdb_port < NUM_CDB_PORTS) begin
                    exec_cdb_grant[i] = 1'b1;
                    cdb_out[cdb_port] = exec_cdb_data[i];
                    cdb_port++;
                end
            end
        end
    end
endgenerate
endmodule