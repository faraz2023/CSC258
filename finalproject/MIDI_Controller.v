module MIDI_Controller(SW, KEY, CLOCK_50, PS2_CLK, PS2_DAT, CLOCK2_50, FPGA_I2C_SCLK, FPGA_I2C_SDAT, AUD_XCK, 
		        AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK, AUD_ADCDAT, AUD_DACDAT, VGA_CLK,VGA_HS, VGA_VS, VGA_BLANK_N,
				  VGA_SYNC_N, VGA_R, VGA_G, VGA_B);


input [9:0] SW;
input [3:0] KEY;

input CLOCK_50;
input CLOCK2_50;

output FPGA_I2C_SCLK;
inout FPGA_I2C_SDAT;
output AUD_XCK;
input AUD_DACLRCK;
input AUD_ADCLRCK;
input AUD_BCLK;
input AUD_ADCDAT; 
output AUD_DACDAT;

input PS2_CLK;
input PS2_DAT;


output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK_N;				//	VGA BLANK
output			VGA_SYNC_N;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]


wire speaker;

wire valid;	// 0 if ket is pressed or released
wire makeBreak;	// 1 for make, 0 for break
wire [4:0] keyboard;

wire enable;

wire displayPlot;



audio_interface audio(
	
		.speaker(speaker) , 
		.cLOCK_50(CLOCK_50) , 
		.cLOCK2_50(CLOCK2_50) , 
		.kEY(KEY[0]) , 
		.fPGA_I2C_SCLK(FPGA_I2C_SCLK) , 
		.fPGA_I2C_SDAT(FPGA_I2C_SDAT) , 
		.aUD_XCK(AUD_XCK) , 
		.aUD_DACLRCK(AUD_DACLRCK) , 
		.aUD_ADCLRCK(AUD_ADCLRCK) , 
		.aUD_BCLK(AUD_BCLK) , 
		.aUD_ADCDAT(AUD_ADCDAT) , 
		.aUD_DACDAT(AUD_DACDAT)
	);
	
	
piano p1(

		.scale(SW[3:0]) ,
		.octave(SW[9:7]) ,
		.note(SW[3:0]) ,
		.speaker(speaker) ,
		.clk(CLOCK_50) ,
		.reset(KEY[0]) ,
		.enable(enable)
	);
	
	
keyboard k1(

		.out(keyboard) ,
		.pS2_DAT(PS2_DAT) ,
		.pS2_CLK(PS2_CLK) ,
		.reset(KEY[0]) ,
		.clk(CLOCK_50) ,
		.EN(enable) ,
		.plot(displayPlot)
	);
	

	
display d1(
		.cLOCK_50(CLOCK_50) ,	//	On Board 50 MHz
		.note(keyboard) ,
		.octave(SW[9:7]) ,
		.writeEN(displayPlot) ,
		.colour(3'b100) ,	// Red
		.resetn(KEY[0]) ,
		// The ports below are for the VGA output.  Do not change.
		.vGA_CLK(VGA_CLK) ,   		//	VGA Clock
		.vGA_HS(VGA_HS) ,				//	VGA H_SYNC
		.vGA_VS(VGA_VS) ,				//	VGA V_SYNC
		.vGA_BLANK_N(VGA_BLANK_N) ,		//	VGA BLANK
		.vGA_SYNC_N(VGA_SYNC_N) ,			//	VGA SYNC
		.vGA_R(VGA_R) ,   			//	VGA Red[9:0]
		.vGA_G(VGA_G) ,	 			//	VGA Green[9:0]
		.vGA_B(VGA_B)   				//	VGA Blue[9:0]
	);


endmodule 



////////////////////////////////////////////////////////////////////////////////////////////////


module keyboard(out, pS2_DAT, pS2_CLK, reset, clk, EN, plot);

output reg [4:0] out;

input pS2_DAT, pS2_CLK;

input reset, clk;

output reg EN;

output plot;



wire resetHigh = ~reset;

wire valid, makeBreak;

wire [7:0] keyCode;

reg [11:0] keys;


keyboard_press_driver kpd1(

		.cLOCK_50(clk) , 
		.valid(valid) , 
		.makeBreak(makeBreak) ,
		.outCode(keyCode) ,
		.pS2_DAT(pS2_DAT) , // PS2 data line
		.pS2_CLK(pS2_CLK) , // PS2 clock line
		.reset(resetHigh)
);


always @(*) begin

	EN = 1'b1;	// Default
	
	if (makeBreak == 1'b1 && valid == 1'b1) begin
		
		keys[11:0] = 12'b0000_0000_0000; // Just for now, so that only one note plays at a time
	
		case(keyCode)
			8'h1C : keys[0] = 1'b1;	// A
			8'h1D : keys[1] = 1'b1;	// W
			8'h1B : keys[2] = 1'b1;	// S
			8'h24 : keys[3] = 1'b1;	// E
			8'h23 : keys[4] = 1'b1;	// D
			8'h2B : keys[5] = 1'b1;	// F
			8'h2C : keys[6] = 1'b1;	// T
			8'h34 : keys[7] = 1'b1;	// G
			8'h35 : keys[8] = 1'b1;	// Y
			8'h33 : keys[9] = 1'b1;	// H
			8'h3C : keys[10] = 1'b1;	// U
			8'h3B : keys[11] = 1'b1;	// J
			default : keys[0] = keys[0];
		endcase
	end
	
	else if (makeBreak == 1'b0 && valid == 1'b1) begin		
		case(keyCode)
			8'h1C : keys[0] = 1'b0;	// A
			8'h1D : keys[1] = 1'b0;	// W
			8'h1B : keys[2] = 1'b0;	// S
			8'h24 : keys[3] = 1'b0;	// E
			8'h23 : keys[4] = 1'b0;	// D
			8'h2B : keys[5] = 1'b0;	// F
			8'h2C : keys[6] = 1'b0;	// T
			8'h34 : keys[7] = 1'b0;	// G
			8'h35 : keys[8] = 1'b0;	// Y
			8'h33 : keys[9] = 1'b0;	// H
			8'h3C : keys[10] = 1'b0;	// U
			8'h3B : keys[11] = 1'b0;	// J
			default : keys[0] = keys[0];
		endcase
	end
	
	
	case(keys)
		12'b0000_0000_0000 : EN = 1'b0;	// No key is pressed, nothing should be played
		12'b0000_0000_0001 : out = 4'b0001;
		12'b0000_0000_0010 : out = 4'b0010;
		12'b0000_0000_0100 : out = 4'b0011;
		12'b0000_0000_1000 : out = 4'b0100;
		12'b0000_0001_0000 : out = 4'b0101;
		12'b0000_0010_0000 : out = 4'b0110;
		12'b0000_0100_0000 : out = 4'b0111;
		12'b0000_1000_0000 : out = 4'b1000;
		12'b0001_0000_0000 : out = 4'b1001;
		12'b0010_0000_0000 : out = 4'b1010;
		12'b0100_0000_0000 : out = 4'b1011;
		12'b1000_0000_0000 : out = 4'b1100;
		default : EN = 1'b0;
	endcase
		
end


assign plot = valid;

endmodule


////////////////////////////////////////////////////////////////////////////////////////////////


module piano(scale, octave, speaker, note, clk, reset, enable);

input [3:0] scale;	// SW[3:0] // not needed for part 1
input [2:0] octave;	// SW[9:7]
input [4:0] note;
input clk;
input reset;
input enable;

output speaker;


localparam c0 = 7'd3058104/1'd2;

localparam frequency = 50000000/440/2;

//wire frequency = (c0/((2'b10)**octave))/((2'b10)**(note/4'b1100));

reg speak;


//a counter that starts at 2 and negates speaker every time it's strictly greater than frequency

reg count;


always @(posedge clk) begin
	
		if ((reset == 1'b0) || (enable == 1'b0)) begin
			speak <= 1'b0;
			count <= 1'b0;
		end
		
		else 
			begin
			
			if (count == frequency) begin
				speak <= ~speak;
				count <= 1'b0;
			end
			
			else
				count <= count + 1'b1;
		end
end

assign speaker = speak;

endmodule 


////////////////////////////////////////////////////////////////////////////////////////////////


module display
	(
		cLOCK_50,						//	On Board 50 MHz
		note,
		octave,
		writeEN,
		colour,
		resetn,
		// The ports below are for the VGA output.  Do not change.
		vGA_CLK,   						//	VGA Clock
		vGA_HS,							//	VGA H_SYNC
		vGA_VS,							//	VGA V_SYNC
		vGA_BLANK_N,						//	VGA BLANK
		vGA_SYNC_N,						//	VGA SYNC
		vGA_R,   						//	VGA Red[9:0]
		vGA_G,	 						//	VGA Green[9:0]
		vGA_B   						//	VGA Blue[9:0]
	);

	input			cLOCK_50;				//	50 MHz

	input [4:0] note;
	input [3:0] octave;
	input writeEN;	
	input [2:0] colour;
	input resetn;
	

	// Do not change the following outputs
	output			vGA_CLK;   				//	VGA Clock
	output			vGA_HS;					//	VGA H_SYNC
	output			vGA_VS;					//	VGA V_SYNC
	output			vGA_BLANK_N;				//	VGA BLANK
	output			vGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	vGA_R;   				//	VGA Red[9:0]
	output	[9:0]	vGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	vGA_B;   				//	VGA Blue[9:0]
	
	
	wire [7:0] x;
	wire [6:0] y;
	
	assign x = (8'b1001_1111/8'b0000_1100) * note;	// map into 12 parts for the 12 notes
	assign y = (7'b111_0111/7'b000_0111) * octave;	// map into 7 parts for the 7 octaves

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(cLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(vGA_R),
			.VGA_G(vGA_G),
			.VGA_B(vGA_B),
			.VGA_HS(vGA_HS),
			.VGA_VS(vGA_VS),
			.VGA_BLANK(vGA_BLANK_N),
			.VGA_SYNC(vGA_SYNC_N),
			.VGA_CLK(vGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
		
endmodule 