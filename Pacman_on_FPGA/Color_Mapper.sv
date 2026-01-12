//-------------------------------------------------------------------------
//    Color_Mapper.sv                                                    --
//    Stephen Kempf                                                      --
//    3-1-06                                                             --
//                                                                       --
//    Modified by David Kesler  07-16-2008                               --
//    Translated by Joe Meng    07-07-2013                               --
//    Modified by Zuofu Cheng   08-19-2023                               --
//                                                                       --
//    Fall 2023 Distribution                                             --
//                                                                       --
//    For use with ECE 385 USB + HDMI                                    --
//    University of Illinois ECE Department                              --
//-------------------------------------------------------------------------

module color_mapper (
    input  logic [9:0] PacmanX, PacmanY, DrawX, DrawY,
    input  logic [9:0] BlueghostX, BlueghostY,
    input  logic [3:0] mazeinr, mazeing, mazeinb,
    input  logic [3:0] pacman_red, pacman_green, pacman_blue,
    input  logic [3:0] blueghost_red, blueghost_green, blueghost_blue,
    output logic [3:0] Red, Green, Blue
);
    logic debug_border;
    assign debug_border = (DrawX < 1) || (DrawX > 639) || (DrawY < 1) || (DrawY > 479);
    
    logic pacman_sprite_region;
    localparam PacmanS = 16;
    assign pacman_sprite_region = (DrawX >= PacmanX - PacmanS/2) && (DrawX < PacmanX + PacmanS/2) && 
                                  (DrawY >= PacmanY - PacmanS/2) && (DrawY < PacmanY + PacmanS/2);
    logic pacman_visible;
    assign pacman_visible = (pacman_red != 0) || (pacman_green != 0) || (pacman_blue != 0);
    
    logic blueghost_sprite_region;
    localparam BlueghostS = 16;
    assign blueghost_sprite_region = (DrawX >= BlueghostX - BlueghostS/2) && (DrawX < BlueghostX + BlueghostS/2) &&
                                     (DrawY >= BlueghostY - BlueghostS/2) && (DrawY < BlueghostY + BlueghostS/2);
    logic blueghost_visible;
    assign blueghost_visible = (blueghost_red != 0) || (blueghost_green != 0) || (blueghost_blue != 0);
   
    always_comb begin
        Red = mazeinr;
        Green = mazeing;
        Blue = mazeinb;
        
        if (blueghost_sprite_region && blueghost_visible) begin
            Red = blueghost_red;
            Green = blueghost_green;
            Blue = blueghost_blue;
        end
        if (pacman_sprite_region && pacman_visible) begin
            Red = pacman_red;
            Green = pacman_green;
            Blue = pacman_blue;
        end
        if (debug_border) begin
            Red = 4'h0;    
            Green = 4'h0;
            Blue = 4'h0;
        end
    end
    
endmodule