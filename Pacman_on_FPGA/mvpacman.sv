`timescale 1ns / 1ps
module mvpacman(
    input  logic        Reset, 
    input  logic        frame_clk,
    input  logic [7:0]  keycode,
    input  logic [9:0]  vga_x,
    input  logic [9:0]  vga_y,
    output logic [3:0]  pacman_red,
    output logic [3:0]  pacman_green,
    output logic [3:0]  pacman_blue,
    output logic [9:0]  PacmanX, 
    output logic [9:0]  PacmanY,
    output logic        game_over
);
    logic [7:0]  tile_value_tl; // Top-left corner
    logic [7:0]  tile_value_tr; // Top-right corner
    logic [7:0]  tile_value_bl; // Bottom-left corner
    logic [7:0]  tile_value_br; // Bottom-right corner
    logic        collision_detected; // Debug signal
    logic [5:0] check_tile_row_tl, check_tile_col_tl;
    logic [5:0] check_tile_row_tr, check_tile_col_tr;
    logic [5:0] check_tile_row_bl, check_tile_col_bl;
    logic [5:0] check_tile_row_br, check_tile_col_br;
    logic [9:0] check_x_tl;
    logic [9:0] check_y_tl;
    logic [9:0] check_x_tr;
    logic [9:0] check_y_tr;
    logic [9:0] check_x_bl;
    logic [9:0] check_y_bl;
    logic [9:0] check_x_br;
    logic [9:0] check_y_br;
    
    parameter [9:0] Pacman_X_Center=320;  // Center position on the X axis
    parameter [9:0] Pacman_Y_Center=296;  // Center position on the Y axis
    parameter [9:0] Pacman_X_Min=0;       // Leftmost point on the X axis
    parameter [9:0] Pacman_X_Max=639;     // Rightmost point on the X axis
    parameter [9:0] Pacman_Y_Min=0;       // Topmost point on the Y axis
    parameter [9:0] Pacman_Y_Max=479;     // Bottommost point on the Y axis
    parameter [9:0] Pacman_X_Step=1;      // Step size on the X axis
    parameter [9:0] Pacman_Y_Step=1;      // Step size on the Y axis

    logic [9:0] Pacman_X_Motion;
    logic [9:0] Pacman_X_Motion_next;
    logic [9:0] Pacman_Y_Motion;
    logic [9:0] Pacman_Y_Motion_next;
    logic [9:0] Pacman_X_next;
    logic [9:0] Pacman_Y_next;
    
    localparam PacmanS = 16;
    
    localparam [7:0] MOVE_TILE = 8'h00;
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
    
    logic [7:0] maze_movement [0:29][0:39];
        
    initial begin
    for (int row = 0; row < 30; row++)
        for (int col = 0; col < 40; col++)
            maze_movement[row][col] = TILE_EMPT2;
        $readmemh("maze2.mem", maze_movement);
    end 
    
    initial begin
        tile_value_tl = TILE_WALL;
        tile_value_tr = TILE_WALL;
        tile_value_bl = TILE_WALL;
        tile_value_br = TILE_WALL;
    end

    always_comb begin
        Pacman_Y_Motion_next = Pacman_Y_Motion; 
        Pacman_X_Motion_next = Pacman_X_Motion;
        collision_detected = 1'b0;
        
        if (keycode == 8'h1A) // W
        begin
            Pacman_Y_Motion_next = -10'd1;
            Pacman_X_Motion_next = 10'b0000000000;
        end
        else if(keycode == 8'h16) // S
        begin
            Pacman_Y_Motion_next = 10'd1;
            Pacman_X_Motion_next = 10'b0000000000;
        end
        else if(keycode == 8'h7) // D
        begin
            Pacman_X_Motion_next = 10'd1;
            Pacman_Y_Motion_next = 10'b0000000000;
        end
        else if(keycode == 8'h4) // A
        begin
            Pacman_X_Motion_next = -10'd1;
            Pacman_Y_Motion_next = 10'b0000000000;
        end
        
        Pacman_X_next = PacmanX + Pacman_X_Motion_next;
        Pacman_Y_next = PacmanY + Pacman_Y_Motion_next;
        check_x_tl = Pacman_X_next - PacmanS/2;
        check_y_tl = Pacman_Y_next - PacmanS/2;
        check_x_tr = Pacman_X_next + PacmanS/2 - 1;
        check_y_tr = Pacman_Y_next - PacmanS/2;
        check_x_bl = Pacman_X_next - PacmanS/2;
        check_y_bl = Pacman_Y_next + PacmanS/2 - 1;
        check_x_br = Pacman_X_next + PacmanS/2 - 1;
        check_y_br = Pacman_Y_next + PacmanS/2 - 1;
        
        if (Pacman_X_Motion_next < 0) begin 
            check_x_tl = check_x_tl - 1;
            check_x_bl = check_x_bl - 1;
        end
        else if (Pacman_X_Motion_next > 0) begin 
            check_x_tr = check_x_tr;
            check_x_br = check_x_br;
        end
        
        if (Pacman_Y_Motion_next < 0) begin 
            check_y_tl = check_y_tl - 1;
            check_y_tr = check_y_tr - 1;
        end
        else if (Pacman_Y_Motion_next > 0) begin 
            check_y_bl = check_y_bl;
            check_y_br = check_y_br;
        end

        check_tile_row_tl = check_y_tl / TILE_HEIGHT;
        check_tile_col_tl = check_x_tl / TILE_WIDTH;
        check_tile_row_tr = check_y_tr / TILE_HEIGHT;
        check_tile_col_tr = check_x_tr / TILE_WIDTH;
        check_tile_row_bl = check_y_bl / TILE_HEIGHT;
        check_tile_col_bl = check_x_bl / TILE_WIDTH;
        check_tile_row_br = check_y_br / TILE_HEIGHT;
        check_tile_col_br = check_x_br / TILE_WIDTH;
        
        tile_value_tl = TILE_WALL;
        tile_value_tr = TILE_WALL;
        tile_value_bl = TILE_WALL;
        tile_value_br = TILE_WALL;

        if(check_tile_row_tl < 30 && check_tile_col_tl < 40) begin
            tile_value_tl = maze_movement[check_tile_row_tl][check_tile_col_tl];
        end

        if(check_tile_row_tr < 30 && check_tile_col_tr < 40) begin
            tile_value_tr = maze_movement[check_tile_row_tr][check_tile_col_tr];
        end

        if(check_tile_row_bl < 30 && check_tile_col_bl < 40) begin
            tile_value_bl = maze_movement[check_tile_row_bl][check_tile_col_bl];
        end

        if(check_tile_row_br < 30 && check_tile_col_br < 40) begin
            tile_value_br = maze_movement[check_tile_row_br][check_tile_col_br];
        end
        
        if (Pacman_X_Motion_next != 0) begin 
            if (Pacman_X_Motion_next == 10'd1) begin // Right
                if ((tile_value_tr == TILE_WALL || tile_value_tr == TL_SQUARE || tile_value_tr == T_RECTANGLE || 
                     tile_value_tr == TR_SQUARE || tile_value_tr == R_RECTANGLE || tile_value_tr == BR_SQUARE || 
                     tile_value_tr == B_RECTANGLE || tile_value_tr == BL_SQUARE || tile_value_tr == L_RECTANGLE ||
                     tile_value_tr == TL_L || tile_value_tr == TR_L || tile_value_tr == BR_L || tile_value_tr == BL_L) ||
                    (tile_value_br == TILE_WALL || tile_value_br == TL_SQUARE || tile_value_br == T_RECTANGLE || 
                     tile_value_br == TR_SQUARE || tile_value_br == R_RECTANGLE || tile_value_br == BR_SQUARE || 
                     tile_value_br == B_RECTANGLE || tile_value_br == BL_SQUARE || tile_value_br == L_RECTANGLE ||
                     tile_value_br == TL_L || tile_value_br == TR_L || tile_value_br == BR_L || tile_value_br == BL_L)) begin
                    Pacman_X_Motion_next = 0;
                    collision_detected = 1'b1;
                end
            end
            else if (Pacman_X_Motion_next == -10'd1) begin // Left
                if ((tile_value_tl == TILE_WALL || tile_value_tl == TL_SQUARE || tile_value_tl == T_RECTANGLE || 
                     tile_value_tl == TR_SQUARE || tile_value_tl == R_RECTANGLE || tile_value_tl == BR_SQUARE || 
                     tile_value_tl == B_RECTANGLE || tile_value_tl == BL_SQUARE || tile_value_tl == L_RECTANGLE ||
                     tile_value_tl == TL_L || tile_value_tl == TR_L || tile_value_tl == BR_L || tile_value_tl == BL_L) ||
                    (tile_value_bl == TILE_WALL || tile_value_bl == TL_SQUARE || tile_value_bl == T_RECTANGLE || 
                     tile_value_bl == TR_SQUARE || tile_value_bl == R_RECTANGLE || tile_value_bl == BR_SQUARE || 
                     tile_value_bl == B_RECTANGLE || tile_value_bl == BL_SQUARE || tile_value_bl == L_RECTANGLE ||
                     tile_value_bl == TL_L || tile_value_bl == TR_L || tile_value_bl == BR_L || tile_value_bl == BL_L)) begin
                    Pacman_X_Motion_next = 0;
                    collision_detected = 1'b1;
                end
            end
        end
        else if (Pacman_Y_Motion_next != 0) begin 
            if (Pacman_Y_Motion_next == -10'd1) begin // Up
                if ((tile_value_tl == TILE_WALL || tile_value_tl == TL_SQUARE || tile_value_tl == T_RECTANGLE || 
                     tile_value_tl == TR_SQUARE || tile_value_tl == R_RECTANGLE || tile_value_tl == BR_SQUARE || 
                     tile_value_tl == B_RECTANGLE || tile_value_tl == BL_SQUARE || tile_value_tl == L_RECTANGLE ||
                     tile_value_tl == TL_L || tile_value_tl == TR_L || tile_value_tl == BR_L || tile_value_tl == BL_L) ||
                    (tile_value_tr == TILE_WALL || tile_value_tr == TL_SQUARE || tile_value_tr == T_RECTANGLE || 
                     tile_value_tr == TR_SQUARE || tile_value_tr == R_RECTANGLE || tile_value_tr == BR_SQUARE || 
                     tile_value_tr == B_RECTANGLE || tile_value_tr == BL_SQUARE || tile_value_tr == L_RECTANGLE ||
                     tile_value_tr == TL_L || tile_value_tr == TR_L || tile_value_tr == BR_L || tile_value_tr == BL_L)) begin
                    Pacman_Y_Motion_next = 0;
                    collision_detected = 1'b1;
                end
            end
            else if (Pacman_Y_Motion_next == 10'd1) begin // Down
                if ((tile_value_bl == TILE_WALL || tile_value_bl == TL_SQUARE || tile_value_bl == T_RECTANGLE || 
                     tile_value_bl == TR_SQUARE || tile_value_bl == R_RECTANGLE || tile_value_bl == BR_SQUARE || 
                     tile_value_bl == B_RECTANGLE || tile_value_bl == BL_SQUARE || tile_value_bl == L_RECTANGLE ||
                     tile_value_bl == TL_L || tile_value_bl == TR_L || tile_value_bl == BR_L || tile_value_bl == BL_L || tile_value_bl == GHOST_GATE) ||
                    (tile_value_br == TILE_WALL || tile_value_br == TL_SQUARE || tile_value_br == T_RECTANGLE || 
                     tile_value_br == TR_SQUARE || tile_value_br == R_RECTANGLE || tile_value_br == BR_SQUARE || 
                     tile_value_br == B_RECTANGLE || tile_value_br == BL_SQUARE || tile_value_br == L_RECTANGLE ||
                     tile_value_br == TL_L || tile_value_br == TR_L || tile_value_br == BR_L || tile_value_br == BL_L || tile_value_br == GHOST_GATE)) begin
                    Pacman_Y_Motion_next = 0;
                    collision_detected = 1'b1;
                end
            end
        end
    end
        
    always_ff @(posedge frame_clk) begin: Move_Pacman
        if (Reset) begin 
            Pacman_Y_Motion <= 10'd0;
            Pacman_X_Motion <= 10'd0;
            PacmanY <= Pacman_Y_Center;
            PacmanX <= Pacman_X_Center;
            game_over <= 0;
        end
        else if (game_over) begin
            Pacman_Y_Motion <= 0;
            Pacman_X_Motion <= 0;
        end
        else begin 
            Pacman_Y_Motion <= Pacman_Y_Motion_next; 
            Pacman_X_Motion <= Pacman_X_Motion_next; 
            if (Pacman_X_Motion_next != 0 || Pacman_Y_Motion_next != 0) begin
                PacmanY <= Pacman_Y_next;
                PacmanX <= Pacman_X_next;
            end
        end  
    end    
    
    logic [11:0] draw_color;
    logic pacman_cur_pixel;
    localparam [7:0] PIXEL_EMPTY = 8'h00;
    localparam [7:0] PIXEL_YELLOW  = 8'h01;
    logic [3:0] pacman_row;
    logic [3:0] pacman_col;
    logic sprite_region;
    
    logic [7:0] pacman_sprite [0:15][0:15] = '{
        '{8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h00, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00},
        '{8'h00, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00},
        '{8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h00, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h01, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00},
        '{8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00}
    };
    
    always_comb begin
        pacman_red = 4'h0;
        pacman_green = 4'h0;
        pacman_blue = 4'h0;
        
        sprite_region = (vga_x >= PacmanX - PacmanS/2) && (vga_x < PacmanX + PacmanS/2) && 
                        (vga_y >= PacmanY - PacmanS/2) && (vga_y < PacmanY + PacmanS/2);
        
        if (sprite_region) begin
            pacman_row = vga_y - (PacmanY - PacmanS/2);
            pacman_col = vga_x - (PacmanX - PacmanS/2);
            
            pacman_cur_pixel = pacman_sprite[pacman_row][pacman_col];
            
            case(pacman_cur_pixel)
                PIXEL_EMPTY: begin
                    // do nothing: transparent pixel
                end
                PIXEL_YELLOW: begin
                    pacman_red = 4'hF;
                    pacman_green = 4'hF;
                    pacman_blue = 4'h0;
                end
                default: begin
                    pacman_red = 4'hF;
                    pacman_green = 4'hF;
                    pacman_blue = 4'hF;
                end
            endcase
        end
    end
   
endmodule