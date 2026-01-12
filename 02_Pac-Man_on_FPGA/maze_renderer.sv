`timescale 1ns / 1ps
module maze_renderer (
    input  logic [9:0] vga_x,
    input  logic [9:0] vga_y,
    output logic [3:0] red,
    output logic [3:0] green,
    output logic [3:0] blue
);
    
    logic [11:0] draw_color;
    
    localparam [7:0] TILE_EMPTY = 8'h00;
    localparam [7:0] TILE_WALL  = 8'h01;
    localparam [7:0] TILE_DOT   = 8'h02;
    
    localparam [7:0] TL_SQUARE = 8'h10;
    localparam [7:0] T_RECTANGLE = 8'h11;
    localparam [7:0] TR_SQUARE = 8'h12;
    localparam [7:0] R_RECTANGLE = 8'h13;
    localparam [7:0] BR_SQUARE = 8'h14;
    localparam [7:0] B_RECTANGLE = 8'h15;
    localparam [7:0] BL_SQUARE = 8'h16;
    localparam [7:0] L_RECTANGLE = 8'h17;
    localparam [7:0] TL_L = 8'h18;
    localparam [7:0] TR_L = 8'h19;
    localparam [7:0] BR_L = 8'h20;
    localparam [7:0] BL_L = 8'h21;
    localparam [7:0] GHOST_GATE = 8'h22;
    localparam [7:0] TILE_EMPT2 = 8'h25;
    
    localparam TILE_WIDTH  = 16;
    localparam TILE_HEIGHT = 16;

    logic [7:0] maze_map [0:29][0:39];
    
    
    
    initial begin
        for (int row = 0; row < 30; row++)
            for (int col = 0; col < 40; col++)
                maze_map[row][col] = TILE_EMPTY;
                
        $readmemh("mazeansi.mem", maze_map);
    end
    
    logic [5:0] tile_row;
    logic [5:0] tile_col;
    logic [3:0] px;
    logic [3:0] py;
    logic [7:0] tile_value;
    
    logic signed [4:0] dx, dy;
    
    always_comb begin
        tile_row = vga_y / TILE_HEIGHT;
        tile_col = vga_x / TILE_WIDTH;
        px = vga_x % TILE_WIDTH;
        py = vga_y % TILE_HEIGHT;
        
        if (tile_row < 30 && tile_col < 40)
            tile_value = maze_map[tile_row][tile_col];
        else
            tile_value = TILE_EMPTY;
    
        case (tile_value)
            TILE_EMPTY: draw_color = 12'h000; 
            TILE_WALL: draw_color = 12'h00F; 
            TILE_DOT: begin
                        dx = px - 8;
                        dy = py - 8;
                        draw_color = ((dx * dx) + (dy * dy) <= 4) ? 12'hFFF : 12'h000;
                      end
            TL_SQUARE: draw_color = ((px >= 8 && px <= 15) && (py >= 8 && py <= 15)) ? 12'h00F : 12'h000;
            T_RECTANGLE: draw_color = (py >= 8 && py <= 15) ? 12'h00F : 12'h000;
            TR_SQUARE: draw_color = ((px >= 0 && px <= 7) && (py >= 8 && py <= 15)) ? 12'h00F : 12'h000;
            R_RECTANGLE: draw_color = (px >= 0 && px <= 7) ? 12'h00F : 12'h000;
            BR_SQUARE: draw_color = ((px >= 0 && px <= 7) && (py >= 0 && py <= 7)) ? 12'h00F : 12'h000;
            B_RECTANGLE: draw_color = (py >= 0 && py <= 7) ? 12'h00F : 12'h000;
            BL_SQUARE: draw_color = ((px >= 8 && px <= 15) && (py >= 0 && py <= 7)) ? 12'h00F : 12'h000;
            L_RECTANGLE: draw_color = (px >= 8 && px <= 15) ? 12'h00F : 12'h000;
            TL_L: draw_color = ((px >= 8 && px <= 15) && (py >= 8 && py <= 15)) ? 12'h000 : 12'h00F;
            TR_L: draw_color = ((px >= 0 && px <= 7) && (py >= 8 && py <= 15)) ? 12'h000 : 12'h00F;
            BR_L: draw_color = ((px >= 0 && px <= 7) && (py >= 0 && py <= 7)) ? 12'h000 : 12'h00F;
            BL_L: draw_color = ((px >= 8 && px <= 15) && (py >= 0 && py <= 7)) ? 12'h000 : 12'h00F;
            GHOST_GATE: draw_color = 12'hFFF;
            TILE_EMPT2: draw_color = 12'h000;
            default: draw_color = 12'hFFF; 
        endcase 
        red = draw_color[11:8];
        green = draw_color[7:4];
        blue = draw_color[3:0];
    end 
    
endmodule