module audio_interface (speaker, cLOCK_50, cLOCK2_50, kEY, fPGA_I2C_SCLK, fPGA_I2C_SDAT, aUD_XCK, 
		        aUD_DACLRCK, aUD_ADCLRCK, aUD_BCLK, aUD_ADCDAT, aUD_DACDAT);

	input cLOCK_50, cLOCK2_50;
	input kEY;
	// I2C Audio/Video config interface
	output fPGA_I2C_SCLK;
	inout fPGA_I2C_SDAT;
	// Audio CODEC
	output aUD_XCK;
	input aUD_DACLRCK, aUD_ADCLRCK, aUD_BCLK;
	input aUD_ADCDAT;
	output aUD_DACDAT;
	// Audio input
	input speaker;
	
	
	// Local wires.
	wire reset = ~kEY;
	wire read_ready, write_ready, read, write;
	wire [23:0] readdata_left, readdata_right;
	wire [23:0] writedata_left, writedata_right;
	

	/////////////////////////////////
	// Your code goes here 
	/////////////////////////////////
	
	
	assign writedata_left = 24'b0000_0000_1111_1111_1111_1111;
	assign writedata_right = 24'b0000_0000_1111_1111_1111_1111;
	
	assign write = speaker;
	
	assign read = 1'b0;
	
/////////////////////////////////////////////////////////////////////////////////
// Audio CODEC interface. 
//
// The interface consists of the following wires:
// read_ready, write_ready - CODEC ready for read/write operation 
// readdata_left, readdata_right - left and right channel data from the CODEC
// read - send data from the CODEC (both channels)
// writedata_left, writedata_right - left and right channel data to the CODEC
// write - send data to the CODEC (both channels)
// AUD_* - should connect to top-level entity I/O of the same name.
//         These signals go directly to the Audio CODEC
// I2C_* - should connect to top-level entity I/O of the same name.
//         These signals go directly to the Audio/Video Config module
/////////////////////////////////////////////////////////////////////////////////
	clock_generator my_clock_gen(
		// inputs
		CLOCK2_50,
		reset,

		// outputs
		AUD_XCK
	);

	audio_and_video_config cfg(
		// Inputs
		CLOCK_50,
		reset,

		// Bidirectionals
		FPGA_I2C_SDAT,
		FPGA_I2C_SCLK
	);

	audio_codec codec(
		// Inputs
		CLOCK_50,
		reset,

		read,	write,
		writedata_left, writedata_right,

		AUD_ADCDAT,

		// Bidirectionals
		AUD_BCLK,
		AUD_ADCLRCK,
		AUD_DACLRCK,

		// Outputs
		read_ready, write_ready,
		readdata_left, readdata_right,
		AUD_DACDAT
	);

endmodule


