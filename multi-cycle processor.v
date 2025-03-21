//-----------------------------------------------------------------------------
//
// Title       : proccessor
// Design      : processor
// Author      : Sarah Hassouneh & Sundus Qasrawi
// Company     : Birzeit University
//
//-----------------------------------------------------------------------------
//
// Description :  Version 5- Connected Ready for testing 
//
//-----------------------------------------------------------------------------

`timescale 1ps / 1ps

/*	
	The ALU full module.
*/
module ALU (ALU_opcode, a, b, result, zero , carry , overflow, negative);	
	
	// define parameters for lengths
	parameter data_width = 16,opcode_length=2;	
	
	input [opcode_length-1:0] ALU_opcode;  
	//input ALU_control; 
	input signed  [data_width-1:0] a, b; // assume inputs and outpust are signed here >> see this 
	output reg signed  [data_width-1:0] result;
	// flags for output 
	output reg zero , carry , overflow, negative; 
	
	
	
	//define parametrs for operations 
	parameter add_op=1, sub_op=2, and_op=0, no_op=3;
	
	//perform correct operation
	always @(*)
    begin 
				
		case (ALU_opcode)
            add_op:
				begin
					{carry,result} = a+b;
										
					negative	= result[data_width-1]; // Set negative flag based on MSB of result because we are using 2's complement
					
					if 	((a[data_width-1] == 0 && b[data_width-1] == 0 && result[data_width-1] == 1) || 
                		(a[data_width-1] == 1 && b[data_width-1] == 1 && result[data_width-1] == 0)) 
               				overflow = 1;
             		else 
						 overflow = 0;
						 
					
				end 
            sub_op:
				begin	
				{carry,result} = a-b;
				negative	= result[data_width-1];
				
				if 	((a[data_width-1] == 0 && b[data_width-1] == 1 && result[data_width-1] == 1) || 
                		(a[data_width-1] == 1 && b[data_width-1] ==0  && result[data_width-1] == 0)) 
               				overflow = 1;
             	else 
						 overflow = 0;
						 
						 
				
				end 
			
		   	and_op:		   
			   begin
				   {carry,result} = a & b;	  // bitwise operation	   
				   	carry		= 0;   
					overflow	= 0;
					negative	= 0;
		   	   end 
			
			no_op:		   
			   begin
				   // do nothing (reserve last outcome)
		   	   end 
				  
				  
				  
            default: 	
				begin 
					result		= 0;
					carry		= 0;   
					overflow	= 0;
					negative	= 0;
				end
			
        endcase	
		
		// set zero flag :
		if (result==0)
			zero=1;
		else 
			zero=0;
		
		end	
		
			
endmodule 







/*	
	This is the main register file. It has 3 adresses, 2 for reads and 1 for write.Only the read is a
	synchronized with the clock. 

*/	
module reg_file (clk,RegWr,addr_read1, addr_read2, addr_write, bus_read1 ,bus_read2, bus_write, reg_Access, reg_Data); 
	
	// define parameters for the memory
	parameter data_width = 16, address_width = 3;
	
	//define inputs in terms of parameters
	input clk; 
	input RegWr;// this is an enable for writing 
	input [(address_width-1):0] addr_read1, addr_read2, addr_write; 
	input signed [(data_width -1):0] bus_write; 
	output reg signed [(data_width -1):0] bus_read1 ,bus_read2; 	 
	
	// define register file in terms of parameters
	reg signed [(data_width-1):0] registers [ 0:((1<<address_width)-1)];
	
	// temp for testing 
	input [(address_width-1):0] reg_Access; 
	output reg signed [(data_width -1):0] reg_Data;
	
	always @(*)
		begin
			reg_Data <= registers[reg_Access];	
		end
	
	initial 
		begin
			
        	initialize_registers; // this is a task to intailize registers  
			bus_read1=0;
			bus_read2=0;
		
    		end
	
	always @(*)
			begin
				// Combinational logic 
				bus_read1 <= registers[addr_read1];	
				bus_read2 <= registers[addr_read2];			
			end
	
	
	reg disable_signal=0; // 0 : do not disable , 1 : disable 
	
	always @(posedge clk) 
	begin
			
		// check if writing on zero -> disable that 
		if (addr_write == 0)
			disable_signal=1;
		else
			disable_signal=0;
		
			
		if(RegWr && !disable_signal ) // if enabled
			begin 
				// write on clock edge
				registers[addr_write] <= bus_write;			
			end
	end 
		
	// Task to initialize registers
	
    task initialize_registers; 
		//registers = '{16'd0,16'd1,16'd2,16'd15,16'd4,16'd5,16'd6,16'd7};
	   	registers[0] = 16'd0;
        registers[1] = 16'd1;
        registers[2] = 16'd2;
        registers[3] = 16'd15; 
        registers[4] = 16'h32F5; 
        registers[5] = 16'd5;
        registers[6] = 16'd6;
        registers[7] = 16'd7;
    endtask
	
	
endmodule 


	

/*	
	The ALU control unit.
*/
module ALU_control (opcode,state, ALU_opcode);	
	
	parameter AND = 4'b0000, ADD = 4'b0001, SUB = 4'b0010, ADDI = 4'b0011, 
			ANDI = 4'b0100, LW = 4'b0101, LBu = 4'b0110 , LBs = 4'b0110,
			SW = 4'b0111, BGT = 4'b1000, BGTZ = 4'b1000, BLT = 4'b1001, BLTZ = 4'b1001, 
			BEQ = 4'b1010, BEQZ = 4'b1010, BNE = 4'b1011, BNEZ = 4'b1011, JMP = 4'b1100, 
			CALL = 4'b1101, RET = 4'b1110, SV = 4'b1111;  
			
			
	 input [3:0] state;  
	 
	parameter instruction_fetch = 0,
	instruction_decode = 1,  
	R_type_ALU = 2,
	R_type_completion = 3,
	I_type_ALU = 4,
	I_type_completion = 5, 
	Address_computation = 6, 
	Load_Mem_Access = 7,
	Store_Mem_Access = 8, 
	Load_completion = 9,
	Branch_Completion = 10, 
	Call_completion = 11, 
	Sv_completion = 12;
	
	
	
	// define parameters for lengths
	parameter opcode_length=4 ,ALU_opcode_length=2;	
	
	input [opcode_length-1:0] opcode;  
	//input ALU_Control;
	output reg [ALU_opcode_length-1:0] ALU_opcode;
	
	
	//define parametrs for operations 
	parameter add_op=1, sub_op=2, and_op=0, no_op=3; 
	
	
	always@(*)
		
		begin  
			ALU_opcode=no_op; // by default 
			
		
			// correct operation based on state
		case (state)
			
			Address_computation: 
					ALU_opcode=add_op;
			
			R_type_ALU, I_type_ALU :		
			begin
				if (opcode == AND || opcode == ANDI)
						ALU_opcode= and_op;
			
				else if (opcode == ADD || opcode ==ADDI )
					ALU_opcode= add_op;
		
				else if (opcode == SUB )
					ALU_opcode= sub_op;
		
				else 
					ALU_opcode= no_op; 	
			end
		
		
			Branch_Completion:
				ALU_opcode=sub_op;
		
		
		 default: ALU_opcode=no_op;
		
		endcase 
		end
			
endmodule 


	

module data_memory (clk,MemRd,MemWr,addr,Data_in,Data_out, read_Data,read_Access ); 
	
    // define parameters for the memory
    parameter data_size = 16, address_width = 16;
    
	//define inputs and outputs in terms of parameters
	input clk;
    input MemRd;
    input MemWr;
    input [(address_width-1):0] addr, read_Access; 
    input signed [(address_width -1):0] Data_in;
	
	// temp for testing 
	output reg  [(data_size -1):0] read_Data; 	 
	
    output reg signed [(data_size -1):0] Data_out; 	 
    
   // Define byte addressable memory  array	
	reg [7:0] memory [0:65535]; // 2^16 * 8-bit cells	
	
	
	
	// initialize data memory 
	initial 
	begin 
		
		{memory[12], memory[13]} = 16'hF590;
		
		//memory[12] = 8'd10;
		//memory[13] = 8'h00;
		
		// Initialize memory addresses from 20 to 30 
        memory[20] = 8'h34;
        memory[21] = 8'h56;
        memory[22] = 8'h78;
        memory[23] = 8'h90;
        memory[24] = 8'h12;
        memory[25] = 8'h23;
        memory[26] = 8'h45;
        memory[27] = 8'h67;
        memory[28] = 8'h89;
        memory[29] = 8'h11;
        memory[30] = 8'h13;
					
	end 
	
	always @(*)
		begin 
			// temp for testing
			read_Data = {memory [read_Access  + 1],  memory[read_Access]};	
		end 
   
		
	// write at positive edge of the clock 
	always @(posedge clk)
	begin 
		
		if (MemWr) begin		   			
			memory[addr] = Data_in[7:0];
			memory[addr + 1] = Data_in[15:8]; 
		end
	end

	always @(addr)
		begin 	 	  
			
			if (MemRd)begin
				// start reading at given addr
				Data_out = {memory [addr + 1],  memory[addr]}; 
			end 
		end	
    
endmodule



module instruction_memory (addr,instruction); 
	
	// define parameters for the memory
	parameter instruction_size = 16, address_width = 3;
	
	//define inputs and outputs in terms of parameters
	input [(address_width-1):0] addr; 
	output reg [(instruction_size -1):0] instruction ; 	 
	
	reg [7:0] memory [0:65535]; // 2^16 * 8-bit cells	

	// initialize memory with instructions
	initial begin 
  		 //memory[0] = 8'b10001000;
		 //memory[1] = 8'b00010110;  	  
		 
		
		 
		 {memory[1], memory[0]} = 16'hC01E; //   J-type:     Jump to address 30 (current PC is less than 0x0FFF)	
		  {memory[31], memory[30]} = 16'hD028; //   J-type:     Call to address 40 (current PC is less than 0x0FFF)		   
		   {memory[41], memory[40]} = 16'hE000; //   J-type:     Return after call
		   
		   
		 //{memory[1], memory[0]} = 16'h634A; //   I-type:    (Load Byte unsigned ) R3 = Mem[R2 + 10]
		 
		 
		 //{memory[3], memory[2]} = 16'h6C4A; //   I-type:     (Load Byte signed ) R4 = Mem[R2 + 10]
		 
		 //{memory[1], memory[0]} = 16'h7626; //   I-type:  (Store) Mem[R1 + 6] = R6
		 
		 //{memory[1], memory[0]} = 16'h744A; //   I-type:  (Store) Mem[R2 + 10] = R4 -> Mem[12]=4
		
		  //{memory[1], memory[0]} = 16'hFC32; //	 S-type	: Mem[R6]=50 
		 
		 //{memory[1], memory[0]} = 16'h1688;	   //add R3,R2,R1
		 
		 //{memory[1], memory[0]} = 16'h554A; //   I-type:   (Load word) R5 = Mem[R2 + 10] ->	 R5 = Mem[12]
		 
		 //{memory[3], memory[2]} = 16'h2B18; //	 SUB R5,R4,R3
		 
		  //{memory[3], memory[2]} = 16'h2AD0; //	 SUB R5,R3,R2
		
		 //{memory[3], memory[2]} = 16'hFC32; //	 S-type	: Mem[R6]=50 
		 
		 //{memory[3], memory[2]} = 16'h744A; //   I-type:  (Store) Mem[R2 + 10] = R4 -> Mem[12]=4
		 
		 //{memory[5], memory[4]} = 16'h7626; //   I-type:  (Store) Mem[R1 + 6] = R6
		 		 
		 //{memory[5], memory[4]} = 16'h554A; //   I-type:   (Load word) R5 = Mem[R2 + 10] ->	 R5 = Mem[12] 
		 
		 
		 //{memory[5], memory[4]} = 16'h634A; //   I-type:    (Load Byte unsigned ) R3 = Mem[R2 + 10]
		 
		 
		 //{memory[5], memory[4]} = 16'h6A4A; //   I-type:     (Load Byte signed ) R2 = Mem[R2 + 10] 
		 
		 
	
		 
		  
		 	//{memory[5], memory[4]} = 16'h8324; // I-type : (BGT) if(R3 > R1) Next PC = PC + 4 else PC = PC + 2
		  
		  	 
			//{memory[5], memory[4]} = 16'h9186; // I-type : (BLT) if (R1< R4) Next PC = PC +6 else PC = PC + 2
		  	
				
			
		   
		   
		   
		   // R-type:
			//{memory[1], memory[0]} = 16'h0D28;
			//{memory[3], memory[2]} = 16'h2B18;
		
			// I-type 
			//{memory[5], memory[4]} = 16'h4688; // R6=R4 & 8
		
			//Jump to address 30
			//{memory[7], memory[6]} = 16'hC01E;
		
		
			//S-type
			//{memory[31], memory[30]} = 16'hF807;	   // saved at adress 30
			//{memory[33], memory[32]} = 16'hF454;
		
			// Load 
			//{memory[35], memory[34]} = 16'h5534;	 // R5 = Mem[R1 + 20]
			// Store 
			//{memory[35], memory[34]} = 16'h7454;	 // Mem[R2 + 20] = R4
				
		
			// Branch if applicaple
			//{memory[37], memory[36]} = 16'h8D08;	 // (BGTZ) if (R5> 0) , Next PC = PC +8, else PC = PC + 2
			
			
			
			
	end 


	always @(addr)
		begin 	 	  
		
		// The instruction is 16 bits and the memory is byte addressable and little endian
		
		instruction = {memory [addr + 1], memory[addr]}	 ;
		
		end	

	
	
endmodule 	





module decode_and_multiplexing_unit (instruction, I_type_imm, addr_read1, addr_read2, addr_write, J_type_imm, S_type_imm, opcode, mode );	
	
	// define parameters for the memory
    parameter instruction_size = 16, address_width =3;

	input [(instruction_size -1):0] instruction;
    output reg signed [4:0] I_type_imm;
	output reg signed [11:0] J_type_imm;
    output reg signed [8:0] S_type_imm;
    output reg [(address_width -1):0] addr_read1, addr_read2;
    output reg [(address_width -1):0] addr_write;
    output reg [3:0] opcode;  	
    output reg mode ;
	
   parameter AND = 4'b0000, ADD = 4'b0001, SUB = 4'b0010, ADDI = 4'b0011, 
			ANDI = 4'b0100, LW = 4'b0101, LBu = 4'b0110 , LBs = 4'b0110,
			SW = 4'b0111, BGT = 4'b1000, BGTZ = 4'b1000, BLT = 4'b1001, BLTZ = 4'b1001, 
			BEQ = 4'b1010, BEQZ = 4'b1010, BNE = 4'b1011, BNEZ = 4'b1011, JMP = 4'b1100, 
			CALL = 4'b1101, RET = 4'b1110, SV = 4'b1111;  
	always @(*)
		begin 
	   opcode = 	instruction[15:12]; 
	   mode = instruction [11]; 
	   I_type_imm = 0;
	   addr_read1 = 0; 	
	   addr_read2 = 0; 
	   addr_write = 0;
	   J_type_imm = 0; 
	   S_type_imm = 0;
	   case (instruction[15:12])
		   
		   AND, ADD, SUB : 
		   begin 
				addr_write = instruction[11:9];
				addr_read1 = instruction[8:6];	 
				addr_read2 = instruction[5:3];
		   end 
		   
			ADDI, ANDI: 
			begin 
				addr_write = instruction[10:8];
				addr_read1 = instruction[7:5];	 
				I_type_imm = instruction[4:0];
			end
			
		LW, LBu, LBs: 
			begin
				addr_write = instruction[10:8];
				addr_read1 = instruction[7:5];
				I_type_imm = instruction[4:0];
			end
			
		BGT,BGTZ,BLT,BLTZ,BEQ,BEQZ,BNE,BNEZ:
		
				if (instruction[11] == 0) //mode = 0 compare with RS1		  
					begin 
						addr_read1 = instruction[10:8];	 
						addr_read2 = instruction[7:5];	
						I_type_imm = instruction[4:0];	
					end
				else //mode = 1 compare with RS2 
					begin
						addr_read1 = instruction[10:8];	 
						addr_read2 = 3'b000;	
						I_type_imm = instruction[4:0];
				end
				
		 JMP: 	 
		 	J_type_imm = instruction[11:0];
		 
		 CALL: 	
		 		begin 
			 		addr_write = 3'b111; 
			 		J_type_imm = instruction[11:0];
			   end
		 
		 RET: 	
		 	addr_read1 = 3'b111;   
		 
		 SV: 
		 	begin
			 	S_type_imm =instruction[8:0];
			 	addr_read1 = instruction[11:9];	 
		 	end
		 
		 SW: 
		 	begin
			 	addr_read2 = instruction[10:8];
				addr_read1 = instruction[7:5];	 
				I_type_imm = instruction[4:0];
			end
		 
		 
		endcase 
		end 
 

endmodule




module PC_control ( opcode,mode, zero,state, overflow, carry, negative, PCsrc, PCWrite);	
	
    parameter opcode_length=4;
	
	parameter AND = 4'b0000, ADD = 4'b0001, SUB = 4'b0010, ADDI = 4'b0011, 
			ANDI = 4'b0100, LW = 4'b0101, LBu = 4'b0110 , LBs = 4'b0110,
			SW = 4'b0111, BGT = 4'b1000, BGTZ = 4'b1000, BLT = 4'b1001, BLTZ = 4'b1001, 
			BEQ = 4'b1010, BEQZ = 4'b1010, BNE = 4'b1011, BNEZ = 4'b1011, JMP = 4'b1100, 
			CALL = 4'b1101, RET = 4'b1110, SV = 4'b1111;  
    
	
	input [(opcode_length -1):0] opcode;
	input zero,overflow,carry, negative;  
	input mode;
	input [3:0] state; 
	
	output reg [1:0] PCsrc;
	output reg PCWrite; 
	
	parameter instruction_fetch = 0,
	instruction_decode = 1,  
	R_type_ALU = 2,
	R_type_completion = 3,
	I_type_ALU = 4,
	I_type_completion = 5, 
	Address_computation = 6, 
	Load_Mem_Access = 7,
	Store_Mem_Access = 8, 
	Load_completion = 9,
	Branch_Completion = 10, 
	Call_completion = 11, 
	Sv_completion = 12; 
	
	always@(*)
		begin  
			PCWrite = 0; 
			PCsrc = 0;
		case (state)
		instruction_decode: begin 
			if (opcode == JMP ) 
				begin 
				PCsrc = 2; 
				PCWrite = 1;
				end	
			 if (opcode == RET)	
				 begin
				 PCsrc = 3;
				 PCWrite = 1; 
				 end
			 end
		Call_completion: begin
						PCsrc = 2;
					 	PCWrite = 1;
						end
		 R_type_completion, I_type_completion, Load_completion, Sv_completion, Store_Mem_Access: 
		 				begin
						PCsrc = 0;
					 	PCWrite = 1;
						end	  
		Branch_Completion: begin
			PCWrite = 1; 
		if ((opcode == BEQ || opcode == BEQZ) && zero == 1) 
			PCsrc =1; 
		else if ((opcode == BNE || opcode == BNEZ) && zero == 0)
			PCsrc = 1; 
		else if ((opcode == BGT || opcode == BGTZ) && zero ==0 && negative == overflow )
			PCsrc = 1; 
		else if ((opcode == BLT || opcode == BLTZ) &&  negative != overflow )
			PCsrc = 1; 
		else 
			PCsrc = 0; 
		end
			 endcase
		end	// always end 
	
	
endmodule


module Main_control ( opcode, mode, clk, IRwrite, Ext, Data_in_sel, ALU_src, Mem_add_sel, WB_sel, MemRd, MemWr, RegWr,Byte_ext_sel, state);	
	
	
	// define parameters for the memory
    parameter opcode_length=4;
	
	
	input [(opcode_length -1):0] opcode; 
	input  mode, clk; 
	output reg IRwrite, Ext, Data_in_sel, ALU_src, Mem_add_sel, MemRd, MemWr, RegWr,Byte_ext_sel;	
	output reg [1:0] WB_sel; 
	
 	output reg [3:0] state; 
	 
	parameter AND = 4'b0000, ADD = 4'b0001, SUB = 4'b0010, ADDI = 4'b0011, 
			ANDI = 4'b0100, LW = 4'b0101, LBu = 4'b0110 , LBs = 4'b0110,
			SW = 4'b0111, BGT = 4'b1000, BGTZ = 4'b1000, BLT = 4'b1001, BLTZ = 4'b1001, 
			BEQ = 4'b1010, BEQZ = 4'b1010, BNE = 4'b1011, BNEZ = 4'b1011, JMP = 4'b1100, 
			CALL = 4'b1101, RET = 4'b1110, SV = 4'b1111;  
    
	parameter instruction_fetch = 0,
	instruction_decode = 1,  
	R_type_ALU = 2,
	R_type_completion = 3,
	I_type_ALU = 4,
	I_type_completion = 5, 
	Address_computation = 6, 
	Load_Mem_Access = 7,
	Store_Mem_Access = 8, 
	Load_completion = 9,
	Branch_Completion = 10, 
	Call_completion = 11, 
	Sv_completion = 12; 	 
	
	initial begin 
	state = instruction_fetch;
	end
	
	always@(posedge clk)
	begin
	case (state)
		instruction_fetch : state <= instruction_decode; 
		
		instruction_decode: begin 
				
		   if (opcode == ADD || opcode == AND || opcode == SUB)
			   state <= R_type_ALU;
		  	
		else if (opcode == ADDI || opcode == ANDI) 
			   state <= I_type_ALU;
			
		else if (opcode == LW || opcode == LBu || opcode == LBs || opcode == SW)
			state <= Address_computation; 								  
			
		else if (opcode == BGT || opcode == BGTZ || opcode == BLT || opcode == BLTZ || opcode == BEQ || opcode == BEQZ || opcode == BNE || opcode == BNEZ) 
			state <=  Branch_Completion; 		   
			
		else if 	(opcode == CALL) 
			state <= Call_completion; 	
			
		else if (opcode == SV) 	  
			state <= Sv_completion; 	
			
		else begin 
			state <= instruction_fetch; 
			end		 
		end
		R_type_ALU : state <= R_type_completion;  
		
		R_type_completion : state <= instruction_fetch;	 
		
		I_type_ALU : state <= I_type_completion; 
		
		I_type_completion : state <= instruction_fetch; 
		
		Address_computation : begin 
							case (opcode)
								LW, LBu, LBs : state <= Load_Mem_Access; 
								SW : state <=  Store_Mem_Access; 
								endcase
							end 
		Load_Mem_Access : state <= Load_completion; 
						 			
		Load_completion : state <= instruction_fetch; 
		
		Store_Mem_Access: state <= instruction_fetch; 
		
		Branch_Completion: state <= instruction_fetch; 	
		
		Call_completion :  state <= instruction_fetch; 
		
		Sv_completion :	 state <= instruction_fetch; 
		
		endcase
	end	 
	always @(*) begin 
	 IRwrite = 0; 
	 Ext = 0; 
	 Data_in_sel = 0;
	 ALU_src = 0;
	 Mem_add_sel = 0;
	 WB_sel = 0;
	 MemRd = 0;
	 MemWr = 0;
	 RegWr = 0;
	 Byte_ext_sel=0; 
		
	case (state)
		instruction_fetch :IRwrite = 1;    
		
		instruction_decode:begin
						if (opcode == ANDI)
						Ext = 1; 	//unsigned extension
						end		 
						
		R_type_ALU:  begin
		            ALU_src = 0;	
				   end 
		R_type_completion: begin
			              RegWr = 1; 
						 WB_sel = 0; 
						end
		I_type_ALU: begin
					ALU_src = 1; 
					end
		
		I_type_completion: begin
			              RegWr = 1; 
						 WB_sel = 0; 
						end	   
		
	Address_computation: begin 
						ALU_src = 1; 
					 end 		  
	
	Load_Mem_Access : begin
					 
					Mem_add_sel = 1; 
					MemRd = 1;
					end   
	
	Store_Mem_Access : begin
					MemWr = 1; 
					Mem_add_sel = 1; 
					Data_in_sel = 0;
		              end
	 
	Load_completion : begin 
					RegWr = 1;
					if (opcode == LW) 
						WB_sel = 2;
					else
						WB_sel = 1; 
						case(mode)
						0: Byte_ext_sel= 1; 
						1: Byte_ext_sel=0;
						endcase
					end 
	
	Branch_Completion : begin 
					  ALU_src = 0; 
					 end 
	
	Call_completion : begin 
					 RegWr = 1;
					 WB_sel = 3; 
		             end 
	Sv_completion : begin 
					MemWr = 1; 
					Mem_add_sel = 0; 
					Data_in_sel = 1;
		            end 
	endcase 
	end
endmodule







//----------------------------------------------Small Components----------------------------------------------------------------

/*	
	This is the adder that is used for finding the branch target address.

*/
module adder (a, b, BTA);
	
	
	// define parameters for lengths
	parameter data_width = 16;		 
	input signed  [data_width-1:0] a, b; // assume inputs and outpust are signed here >> see this 	
	output reg signed  [data_width-1:0] BTA;
	
	always @(*)			
			BTA = a+b;

				
	
endmodule 	

/*	
	This is the adder that is used for fiding the next pc.
	
*/
module adder_2 (a, next_PC);
	
	
	// define parameters for lengths
	parameter data_width = 16;		 
	input [data_width-1:0] a; 
	output reg [data_width-1:0] next_PC;
	
	always @(*)			
			next_PC = a +2; // two because we have 2 bytes instrctions
				
	
endmodule 


module mux4x1 (a0, a1, a2, a3,s, out);	
	
	// parameter 
	parameter bus_width =16;
	
	input [(bus_width-1):0] a0,a1,a2,a3;
	input [1:0] s;
	output reg [(bus_width-1):0]  out;	
	
	always@(*)
	begin
		if (s==0)
			out = a0;
		else if (s==1)
			out =a1;
		else if (s==2)
			out =a2;
		else
			out=a3;
	end
	
endmodule


module mux2x1 (a0, a1,s, out);	
	
	// parameter 
	parameter bus_width =16;
	
	input [(bus_width-1):0] a0,a1;
	input s;
	output reg [(bus_width-1):0]  out;	
	
	always@(*)
	begin
		if (s==0)
			out = a0;
		else
			out=a1;
	end
	
endmodule


module extender (in,sign, out);	
	
	// parameter 
	parameter input_width =5, output_width=16;
	
	input [(input_width-1):0] in;
	input sign;
	output reg [(output_width-1):0]  out;	
	
	always@(*)
	begin
		if (sign == 0)  // Sign extension
      		out = {{output_width-input_width{in[input_width-1]}}, in};
    	
    	else  // Zero extension
      		out = {{{output_width - input_width}{1'b0}},in};
	end
	
endmodule


module byte_extender (in,sign, out);
	
	
	input [15:0] in;
	input sign;
	output reg [(15):0]  out;	
	
	reg [7:0] lower_in;
	
	// takes the lower 8 bits and extends that to 16 bits
  	assign lower_in = in[7:0];
	
	  
	always@(*)
	begin
		if (sign == 0)  // Sign extension
      		out = {{8{lower_in[7]}}, lower_in};
    	
    	else  // Zero extension
      		out = {8'b0, lower_in};
	end
	
endmodule






//------------------------------------------------- Top Module-------------------------------------------------------------	
	
	
/* 
	let's think of this as the top level entity, where a proccessor is composed of the following :
		1.ALU
		2.Register File
		3.Instruction Memory   
		4.Data Memory
		5.Instruction Decode Unit:"A state machine to go to the appropriate state"	 
		6.Control Unit :This includes sub-control units (Main Control, PC control, ALU control) and signal generation.
	
  */
  
module proccessor (clk,ALU_out,state,opcode, IR,PC, PCwrite, ALU_opcode, PCsrc,addr_write,RegWr,immediate , A, B , ALU_result, mem_Data, reg_Data,mem_Access,reg_Access,s,WB_sel, I_type_imm,data_out,bus_write,addr_read1,addr_read2,ALU_src,zero, carry, overflow, negative ); 
	
	parameter instruction_opcode_length=4;
	
	// Parameters for ALU
    parameter data_width = 16, ALU_opcode_length = 2; // data width is the width of general purpose registers.
	// Parameters for reg_file
    parameter reg_address_width = 3;
	// Parameters for instruction_memory and data memory -> (address width is related to size of data and instruction memory)
    parameter instruction_size = 16, address_width = 16;
	// Parameters for PC
	parameter PC_width=16; 
	
	input clk;
	
	output reg [instruction_opcode_length-1:0] opcode; 
	output reg PCwrite;
	output reg signed [(data_width-1):0]ALU_out;
   	output reg [1:0] PCsrc;
	output reg [3:0] state;  
	output reg [(PC_width-1):0] PC;
	
	
	//reg clk;
	output reg [ALU_opcode_length-1:0] ALU_opcode;
	
	// wires and temporary
	wire [(PC_width-1):0] next_PC, BTA, Jump_addr, PC_mux_wire;
	wire [(data_width-1):0] extended_I, extended_S;
	wire [(instruction_size - 1):0] instruction;
	
    //wire signed [4:0] I_type_imm;
	output reg signed  [4:0] I_type_imm;
	
    wire signed [11:0] J_type_imm;
    wire signed [8:0] S_type_imm;
	
	
    //wire[(reg_address_width-1):0] addr_read1,addr_read2 ;
    //wire [(reg_address_width-1):0] addr_write;
	
	output reg [(reg_address_width-1):0] addr_write,addr_read1,addr_read2 ;
	
	wire signed [(data_width-1):0] bus_read1, bus_read2,  ALU_B, data_in_wire, extended_byte;
	wire signed [(PC_width-1):0 ] data_addr_wire;  
	
	
	wire mode;
	
	output reg signed  [(data_width-1):0] data_out,bus_write;
	
	// temp
	output reg signed  [4:0] s;
	assign s  = I_type_imm;
	
	// Flags 
	//wire zero, carry, overflow, negative;
	output reg zero, carry, overflow, negative; 
	
	// Signals
	reg IRwrite,MemRd, MemWr, Ext,Byte_ext_sel, Mem_add_sel, Data_in_sel;
	
	output reg RegWr,ALU_src;
	output reg [1:0] WB_sel; 
	
	
	
	// Special Purpose Registers 
	
	output reg [instruction_size-1:0] IR; // return to this!!!!   
	
	
	// Registers between stages
	output reg [(data_width -1):0]  immediate , A, B , ALU_result;
	
	
	
	initial begin 
		PC = 0; 
	 end 
	 
	
	 
		
	 
	// temp for testing memory
	output reg [(data_width -1):0] mem_Data, reg_Data;
	
	input [(address_width-1):0]mem_Access;	 
	input [2:0]reg_Access;	 
	
	//----------- Datapath Connection: ----------
	
	
	
	
	// Instantiate the instruction_memory 
    instruction_memory #(instruction_size, address_width) instruction_memory (
        .addr(PC),
        .instruction(instruction)
    );
	
	  
   	
	
	// Instantiate the decode_unit module
    decode_and_multiplexing_unit #(instruction_size,address_width) inst_decode_unit (
		.instruction(IR),
		.opcode(opcode),
		.mode(mode),
        .I_type_imm(I_type_imm),
        .J_type_imm(J_type_imm),
        .S_type_imm(S_type_imm),
        .addr_read1(addr_read1),
		.addr_read2(addr_read2),
        .addr_write(addr_write)
		
    );
	
	
	// Instantiate extenders
	extender #(.input_width(5), .output_width(data_width)) I_extender (
    	.in(I_type_imm),
    	.sign(Ext),
    	.out(extended_I)
  	);
	
	  
	extender #( .input_width(9), .output_width(data_width)) S_extender (
    	.in(S_type_imm),
    	.sign(Ext),
    	.out(extended_S)
  	);
	  

	   
	//address for the J-type  (simlar to a concatenation unit)
	assign Jump_addr = {PC[15:12],J_type_imm};
	  
	
	
	 // Instantiate adders :
	adder_2 next_PC_adder (
    	.a(PC),
    	.next_PC(next_PC)
  	); 
	  
	  
	adder BTA_adder (
    	.a(PC),
    	.b(extended_I),
    	.BTA(BTA)
  	);
	 
	
	  
	// Instantiate the reg_file
    reg_file #(data_width, reg_address_width) reg_file (
        .clk(clk),
        .RegWr(RegWr),
        .addr_read1(addr_read1),
        .addr_read2(addr_read2),
        .addr_write(addr_write),
        .bus_write(bus_write),
        .bus_read1(bus_read1),
        .bus_read2(bus_read2),
		.reg_Access(reg_Access),
		.reg_Data(reg_Data)
    );
	
	
	
	// Instantiate mux for ALU B input
	mux2x1 ALU_mux (
    	.a0(B),
    	.a1(immediate),
    	.s(ALU_src),
    	.out(ALU_B)
  	);
	
	  
	  
    // Instantiate the ALU
    ALU #(data_width, ALU_opcode_length) alu (
        .ALU_opcode(ALU_opcode),
        .a(A),
        .b(ALU_B),
        .result(ALU_out),
        .zero(zero),
        .carry(carry),
        .overflow(overflow), 
		.negative(negative)//,
		//.ALU_control(ALU_control)
    );

    
	
	// Instantiate mux for address of data address
	mux2x1 data_address_mux (
    	.a0(A),
    	.a1(ALU_result),
    	.s(Mem_add_sel),
    	.out(data_addr_wire)
  	);
	
	
    
	// Instantiate mux for data_in  of data memory
	mux2x1 data_in_mux (
    	.a0(bus_read2),	// check this
    	.a1(extended_S),
    	.s(Data_in_sel),
    	.out(data_in_wire)
  	);
	
	
    
	
	
	// Instantiate the data_memory module
    data_memory #(data_width, address_width) data_memory (
        .clk(clk),
        .MemRd(MemRd),
        .MemWr(MemWr),
        .addr(data_addr_wire),
        .Data_in(data_in_wire),
        .Data_out(data_out),
		.read_Access(mem_Access),
		.read_Data(mem_Data)
		
    );
	
   
	//Instantiate byte extender module 
	byte_extender inst_byte_extender (
    	.in(data_out),
    	.sign(Byte_ext_sel),
    	.out(extended_byte)
  	);	
   
	
	// Instantiate Bus Write Mux
	mux4x1 write_mux (
    	.a0(ALU_result),
    	.a1(extended_byte),
    	.a2(data_out),
    	.a3(next_PC),// edited here
    	.s(WB_sel),
    	.out(bus_write)
  	);
	
	
	// Instantiate PC Mux
	mux4x1 PC_mux (
    	.a0(next_PC),
    	.a1(BTA),
    	.a2(Jump_addr),
    	.a3(bus_read1), 
    	.s(PCsrc),
    	.out(PC_mux_wire)
  	);
	 
	// Instantiate the ALU control_unit module 
	ALU_control my_ALU (  
    .opcode(opcode), 
	//.ALU_Control(ALU_control),
    .ALU_opcode(ALU_opcode),
	.state(state)
	);


		
	// Instantiate the PC control_unit module 
	PC_control my_PC (
    .opcode(opcode),
	.mode(mode),
	.state(state),
    .zero(zero),
	.overflow(overflow),
	.carry(carry),
	.negative(negative),
	.PCsrc(PCsrc),
	.PCWrite(PCwrite)
	);
	
	
	// Instantiate the main control_unit module 
	Main_control my_main_control (
        .opcode(opcode),
        .IRwrite(IRwrite),
        .Ext(Ext),
	   .Byte_ext_sel(Byte_ext_sel),
        .Data_in_sel(Data_in_sel),
        .ALU_src(ALU_src),	  	 
		//.ALU_control(ALU_control),
        .Mem_add_sel(Mem_add_sel),
        .WB_sel(WB_sel),
        .MemRd(MemRd),
        .MemWr(MemWr),
        .RegWr(RegWr),
		.mode (mode),
		.clk (clk),
		.state(state)
    ); 
	
   
	
  
	
	
 

	
	// Clock synchronization:
	always @ (posedge clk)
		begin
			A <= bus_read1;
			B <= bus_read2;	
			ALU_result <= ALU_out;
			immediate <= extended_I;
			
			if (IRwrite)
				IR <= instruction;
				
			if(PCwrite)
				PC <= PC_mux_wire; 
		end 
	

endmodule

	
//-------------------------------------------------Test top module -------------------------------------------------------------	

module test_processor; 
	
	parameter instruction_opcode_length=4;	
    parameter data_width = 16, ALU_opcode_length = 2; 	
    parameter reg_address_width = 3;
    parameter instruction_size = 16, address_width = 16;
	parameter PC_width=16; 
	
	reg clk; 	
	
	
	wire [instruction_opcode_length-1:0] opcode; 
	wire PCwrite;
	wire signed [(data_width-1):0]ALU_out;
   	wire [1:0] PCsrc;
	wire [3:0] state;  
	wire [(PC_width-1):0] PC, IR;
	wire  [(data_width -1):0]  immediate , A, B , ALU_result;	
	
	// temp for testing 
	wire [(data_width -1):0] mem_Data, data_out,bus_write;	
	reg [(address_width-1):0]mem_Access;	 
	
	wire [(data_width -1):0] reg_Data;	
	reg [2:0]reg_Access;
	
	wire [4:0] s; 
	wire [1:0] WB_sel;
	//wire ALU_control;
	
	wire [4:0] I_type_imm;
	wire RegWr, ALU_src, zero, carry, overflow, negative;
	
	wire [2:0] addr_write,addr_read1, addr_read2 ;
	
	wire [1:0] ALU_opcode;
	
	proccessor myProcessor (.clk(clk),
							.addr_read1(addr_read1),
							.addr_read2(addr_read2),
							.ALU_src(ALU_src),
							.zero(zero),
							.carry(carry),
							.overflow(overflow),
							.negative(negative),
							.ALU_opcode(ALU_opcode),
							.addr_write(addr_write),
							.RegWr(RegWr),
							.ALU_out(ALU_out),
							.state(state),
							.opcode(opcode), 
							.PC(PC), 
							.PCwrite(PCwrite),
							.PCsrc(PCsrc),
							.immediate(immediate),
							.A(A),
							.B(B),
							.ALU_result(ALU_result),
							.mem_Access(mem_Access),
							.mem_Data(mem_Data),
							.reg_Access(reg_Access),
							.reg_Data(reg_Data), 
							.IR(IR),
							.s(s),
							.WB_sel(WB_sel),
							.I_type_imm(I_type_imm),
							.data_out(data_out),
							.bus_write(bus_write)
							
							); 
	
	  
							
	always #5 clk = ~clk; 
	initial 
		begin
			
		#5 
		clk =0;
		
		
		//Start of first Instruction
		#5
		mem_Access=6;
		reg_Access=7;
		
		
	
		#35 // now at 40 nseconds	
		// Start of Second Instruction
		#5
		//mem_Access=12;		
		//reg_Access=4;
		#5
		//reg_Access=4;
		
		#40	  // now at 80 nseconds
		// Start of Third Instruction
		#5
		//mem_Access=0;		
		//reg_Access=7;
		
		
		
		#30	  // now at 110 nseconds
		// Start of Fourth Instruction
		
		
		#20	  // now at 130 nseconds
		// Start of Fifth Instruction
		
		/*#5
		reg_Access=1;
		
		#5
		reg_Access=6;
		
		#5
		mem_Access=7;
		reg_Access=7;
		
		#5
		mem_Access=12;
		reg_Access=7;*/
		
		
		#50 $finish;  
		end
	
	
	
	
		
endmodule 	   


