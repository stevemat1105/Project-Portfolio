`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/09/2025 04:02:09 PM
// Design Name: 
// Module Name: blueghost
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module blueghost(
    input  logic        Reset, 
    input  logic        frame_clk, // vsync
    input  logic [9:0]  vga_x,
    input  logic [9:0]  vga_y,
    output logic [3:0]  blueghost_red,
    output logic [3:0]  blueghost_green,
    output logic [3:0]  blueghost_blue,
    output logic [9:0]  BlueghostX, 
    output logic [9:0]  BlueghostY,
    output logic        game_over
);

    localparam TILE_WIDTH  = 16;
    localparam TILE_HEIGHT = 16;
    localparam MAP_ROWS = 30;
    localparam MAP_COLS = 40;
    
    localparam BlueghostS = 16; 
    parameter [9:0] Blueghost_X_Center = 328; 
    parameter [9:0] Blueghost_Y_Center = 248; 
    parameter [9:0] Blueghost_X_Min = 0;
    parameter [9:0] Blueghost_X_Max = 639;
    parameter [9:0] Blueghost_Y_Min = 0;
    parameter [9:0] Blueghost_Y_Max = 479;
    
    localparam [7:0] MOVE_TILE   = 8'h00;
    localparam [7:0] TILE_WALL   = 8'h01;
    localparam [7:0] TILE_DOT    = 8'h02;
    localparam [7:0] TL_SQUARE   = 8'h10;
    localparam [7:0] T_RECTANGLE = 8'h11;
    localparam [7:0] TR_SQUARE   = 8'h12;
    localparam [7:0] R_RECTANGLE = 8'h13;
    localparam [7:0] BR_SQUARE   = 8'h14;
    localparam [7:0] B_RECTANGLE = 8'h15;
    localparam [7:0] BL_SQUARE   = 8'h16;
    localparam [7:0] L_RECTANGLE = 8'h17;
    localparam [7:0] TL_L        = 8'h18;
    localparam [7:0] TR_L        = 8'h19;
    localparam [7:0] BR_L        = 8'h20;
    localparam [7:0] BL_L        = 8'h21;
    localparam [7:0] GHOST_GATE  = 8'h22;
    localparam [7:0] TILE_EMPT2  = 8'h25;

    logic [7:0] maze_movement [0:MAP_ROWS-1][0:MAP_COLS-1];
    
    initial begin
        for (int row = 0; row < MAP_ROWS; row++)
            for (int col = 0; col < MAP_COLS; col++)
                maze_movement[row][col] = TILE_EMPT2;
        $readmemh("maze2.mem", maze_movement);
    end 

    localparam PATH_LEN = 28;
    logic [9:0] path_x [0:PATH_LEN-1];
    logic [9:0] path_y [0:PATH_LEN-1];
    logic [4:0] path_idx; 

    initial begin
        path_x = '{328, 328, 328, 328, 328, 328, 328, 328, 
                   344, 360, 376, 392, 408, 424,       
                   424, 424, 424, 424, 424, 424, 424, 
                   408, 392, 376, 360, 344, 328, 328}; 
        path_y = '{248, 264, 280, 296, 312, 328, 344, 360, 
                   360, 360, 360, 360, 360, 360,      
                   344, 328, 312, 296, 280, 264, 248,
                   248, 248, 248, 248, 248, 248, 248}; 
    end

    always_ff @(posedge frame_clk or posedge Reset) begin
        if (Reset) begin
            BlueghostX <= Blueghost_X_Center;
            BlueghostY <= Blueghost_Y_Center;
            path_idx <= 0;
            game_over <= 0;
        end else if (game_over) begin
        end else begin
            path_idx <= (path_idx == PATH_LEN-1) ? 0 : path_idx + 1;
            BlueghostX <= path_x[path_idx];
            BlueghostY <= path_y[path_idx];
        end
    end

    logic blueghost_cur_pixel;
    localparam [7:0] PIXEL_EMPTY = 8'h00;
    localparam [7:0] PIXEL_CYAN  = 8'h02;
    logic [3:0] blueghost_row;
    logic [3:0] blueghost_col;
    logic sprite_region;
    
    logic [7:0] blueghost_sprite [0:15][0:15] = '{
        '{8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h00, 8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h02, 8'h02, 8'h00, 8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00, 8'h00, 8'h02, 8'h02, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h02, 8'h02, 8'h00, 8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00, 8'h00, 8'h02, 8'h02, 8'h00, 8'h00},
        '{8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h02, 8'h00, 8'h02, 8'h00, 8'h00, 8'h02, 8'h00, 8'h00, 8'h02, 8'h00, 8'h02, 8'h00, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h00, 8'h00, 8'h02, 8'h00, 8'h00, 8'h02, 8'h00, 8'h00, 8'h02, 8'h00, 8'h00, 8'h00, 8'h02, 8'h00},
        '{8'h00, 8'h02, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h02, 8'h00},
        '{8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00}
    };
    
    always_comb begin
        blueghost_red = 4'h0;
        blueghost_green = 4'h0;
        blueghost_blue = 4'h0;
        
        sprite_region = (vga_x >= BlueghostX - BlueghostS/2) && (vga_x < BlueghostX + BlueghostS/2) && 
                        (vga_y >= BlueghostY - BlueghostS/2) && (vga_y < BlueghostY + BlueghostS/2);
        
        if (sprite_region) begin
            blueghost_row = vga_y - (BlueghostY - BlueghostS/2);
            blueghost_col = vga_x - (BlueghostX - BlueghostS/2);
            
            blueghost_cur_pixel = blueghost_sprite[blueghost_row][blueghost_col];
            
            case(blueghost_cur_pixel)
                PIXEL_EMPTY: begin
                    // Transparent pixel
                end
                PIXEL_CYAN: begin
                    blueghost_red = 4'h0;
                    blueghost_green = 4'hF;
                    blueghost_blue = 4'hF; 
                end
                default: begin
                    blueghost_red = 4'hF;
                    blueghost_green = 4'hF;
                    blueghost_blue = 4'hF; 
                end
            endcase
        end
    end
    
endmodule