module Mem_read(
	input rclk,
	input [31:0]rd_addr,
	output reg[31:0]r_addr
);
always@(posedge rclk)
begin
	r_addr<=rd_addr;
end



endmodule

module instruction_mem(
input winc,wclk,rst,
input [31:0]wdata,r_addr,
output [31:0]rdata
);
reg [31:0]Mem[1023:0];
reg [31:0]w_addr;
assign rdata =Mem[r_addr];
always@(posedge wclk)
begin
	if(rst)
		w_addr<=32'd0;
	else
	begin
	if(winc)
	begin
		Mem[w_addr]<=wdata;
		w_addr<=w_addr+32'd1;
	end
end
end
endmodule

module asynchronous_inst_mem(
input winc,wclk,rclk,rst,
input [31:0]read_addr,wdata,
output [31:0]rdata

);
wire [31:0]r_addr;
instruction_mem inst_mem(.winc(winc),.wclk(wclk),.rst(rst),.wdata(wdata),.r_addr(r_addr),.rdata(rdata));
Mem_read mem_rd(.rclk(rclk),.rd_addr(read_addr),.r_addr(r_addr));
endmodule


module Program_counter_control(clk,branch_status_exe,valid_exe,valid,TAKEN_BRANCH,rst,jump_addr_exe,br_taddr,br_inst_read,br_inst_write,read_sig,write_sig,buffer_select,branch_predictor_select,STALL,prediction_valid_exe,LHT_index_read,LHT_index_write,PC,br_taddr_exe,IF_ID_IR,HALT,instruction);
	parameter JAL=7'b1101111;
	parameter JALR=7'b1100111;
	parameter B_inst=7'b1100011;
	input clk,branch_status_exe,valid_exe,valid,TAKEN_BRANCH,rst;
        input [31:0]jump_addr_exe,br_taddr,instruction;
	output reg [23:0]br_inst_read,br_inst_write;
	output reg read_sig,write_sig,buffer_select,branch_predictor_select,STALL,prediction_valid_exe,HALT;
        output reg [3:0]LHT_index_read,LHT_index_write;
	output reg [31:0]PC,br_taddr_exe;
	output [31:0]IF_ID_IR;
	reg [2:0]inst_rptr,inst_wptr,PC_rptr,PC_wptr;
  	reg [1:0]jump_cond;
  	reg [24:0]instruction_fifo[7:0];
	reg [31:0]PC_fifo[7:0];
//	reg [31:0]Mem[1023:0];//instruction memory
	reg [31:0]PC_reg,PC_prev;
	reg STALL_reg;
	assign IF_ID_IR=instruction;
	always@(*)
	begin
			if(rst)
			begin
				STALL=1'b0;
				PC=32'd0;
			end
			else
			begin
				if(STALL_reg)
					STALL=1'b0;
      			if(valid_exe)//valid data from execution stage-branch target address and jump status
			begin
				if(branch_status_exe!=instruction_fifo[inst_rptr][0])//branch mispredicted
				// STALL=1'b1;//
				begin
				  STALL=1'b1;
        			  if(!(instruction_fifo[inst_rptr][0]))//branch mispredicted - not taken
				  begin
				//	IF_ID_IR<=Mem[br_taddr_exe];
					PC=jump_addr_exe;
				  end
				  else
				  begin
//					IF_ID_IR<=Mem[PC_fifo[PC_rptr]];
              				PC=PC_fifo[PC_rptr];
				    end
		//		  PC_rptr=PC_rptr+3'd1;
				//  STALL<=1'b0;
				  $display("Stall branch_predicted:%d inst_rptr %d PC:%b STALL:%d time:%d",instruction_fifo[inst_rptr],inst_rptr,PC,STALL,$time);

				end
				else
					STALL=1'b0;
            			br_inst_write=(instruction_fifo[inst_rptr][24:1]);
				write_sig=1'b1;//write
				buffer_select=1'b1;//br_inst[0];//choose one of the two BTB
            			branch_predictor_select=1'b1;
				br_taddr_exe=jump_addr_exe;//from exe stage
				prediction_valid_exe=branch_status_exe;//from exe stage
            			LHT_index_write=br_inst_write[3:0];
				
			end
      		//	if(IF_ID_IR[6:0]==B_inst)//conditional jump
			if((instruction[6:0]==B_inst)&&(!STALL))
			begin
				jump_cond=2'b01;//conditional jump
				read_sig=1'b1;//read
				write_sig=valid_exe;
				buffer_select=1'b1;//instruction[7];
            			branch_predictor_select=1'b1;
				br_inst_read=IF_ID_IR[31:8];
            			LHT_index_read=IF_ID_IR[11:8];
				 if(valid&&TAKEN_BRANCH)
				 begin
			      // 	IF_ID_IR<=Mem[br_taddr];
					PC=br_taddr;
								
				 end
				 else
				 begin
	//					IF_ID_IR<=Mem[PC];
					PC=PC_reg;
				end
				
				
                        			
			end
			else if(((instruction[6:0]==JAL)||(instruction[6:0]==JALR))&&(!STALL))
			begin
				jump_cond=2'b10;//unconditional jump
				read_sig=1'b1;//read
				write_sig=valid_exe;
				buffer_select=1'b1;//instruction[7];
				br_inst_read=IF_ID_IR[31:8];
            			branch_predictor_select=1'b0;//Branch TAKEN_BRANCH not required for unconditional jump
				if(valid)
					begin
				//	       	IF_ID_IR<=Mem[br_taddr];
						PC<=br_taddr;
					end
					else
					begin
		        		//	IF_ID_IR<=Mem[PC];
						PC<=PC_reg;
					end
			
			
			end
			else
			begin
				if(!STALL)
				begin
				jump_cond=2'b00;//no jump
				buffer_select=valid_exe;
				branch_predictor_select=valid_exe;
				$display("no jump state jump_cond:%d buffer_select:%d time:%d",jump_cond,buffer_select,$time);
			
				read_sig=1'b0;
				write_sig=valid_exe;
				PC<=PC_reg;
			end

			end
			end
			
		
		
	end
	always@(posedge clk or posedge rst)
	begin
		if(rst)
		begin
			inst_rptr<=3'd0;
          	        inst_wptr<=3'd0;
			PC_rptr<=3'd0;
          	        PC_wptr<=3'd0;

			PC_reg<=32'd0;
			HALT<=1'b0;
		end
		else
		begin
			PC_prev<=PC;
		//	IF_ID_IR<=instruction;

		/*	if((!STALL)&&(jump_cond==2'b00))
			begin

		        	IF_ID_IR<=Mem[PC];

			end*/

			

/*			if(((jump_cond==2'b10)||(jump_cond==2'b01)))
			begin
				if(jump_cond==2'b01)
				instruction_fifo[inst_wptr]<={IF_ID_IR[31:8],TAKEN_BRANCH};
				else
				instruction_fifo[inst_wptr]<={IF_ID_IR[31:8],valid};
				PC_fifo[PC_wptr]<=PC;
				inst_wptr<=inst_wptr+3'd1;
				PC_wptr<=PC_wptr+3'd1;
				$display("cache miss wptr%d time %d",inst_wptr,$time);
			end*/
	/*	if(STALL)//control hazard
			begin
        			  if(!(instruction_fifo[inst_rptr][0]))//branch mispredicted - not taken
				  begin
					IF_ID_IR<=Mem[br_taddr_exe];
					PC<=br_taddr_exe+32'd1;
				end
				  else
				  begin
					IF_ID_IR<=Mem[PC_fifo[PC_rptr]];
              				PC<=PC_fifo[PC_rptr]+32'd1;
				end
		//		  PC_rptr=PC_rptr+3'd1;
				  STALL<=1'b0;
				  $display("Stall branch_predicted:%d inst_rptr %d PC:%b STALL:%d time:%d",instruction_fifo[inst_rptr],inst_rptr,PC,STALL,$time);

			end*/
		        if(STALL)
			begin
				STALL_reg<=1'b1;
				PC_reg<=PC+32'd1;
			end
			else
			begin
				STALL_reg<=1'b0;
			if(((jump_cond==2'b10)||(jump_cond==2'b01)))
			begin
				if(jump_cond==2'b01)
				instruction_fifo[inst_wptr]<={IF_ID_IR[31:8],TAKEN_BRANCH};
				else
				instruction_fifo[inst_wptr]<={IF_ID_IR[31:8],valid};
				PC_fifo[PC_wptr]<=PC_prev+32'd1;
				inst_wptr=inst_wptr+3'd1;
			//	PC_wptr<=PC_wptr+3'd1;
				PC_wptr=inst_wptr;
				$display("cache miss wptr%d time %d",inst_wptr,$time);
			end
				if(jump_cond==2'b00)
				begin
				//	IF_ID_IR<=Mem[PC];
					PC_reg<=PC+32'd1;
				end
				else if(jump_cond==2'b10)
				begin
					if(valid)
					begin
				//	       	IF_ID_IR<=Mem[br_taddr];
						PC_reg<=br_taddr+32'd1;
					end
					else
					begin
		        		//	IF_ID_IR<=Mem[PC];
						PC_reg<=PC+32'd1;
					end
				end	
				else if(jump_cond==2'b01)
				begin
					 if(valid&&TAKEN_BRANCH)
					 begin
					      // 	IF_ID_IR<=Mem[br_taddr];
						PC_reg<=br_taddr+32'd1;
									
					 end
	  				 else
					 begin
	//					IF_ID_IR<=Mem[PC];
						PC_reg<=PC+32'd1;
					end
            			 end

				 $display("PC updation jump_cond:%d PC:%b valid:%d br_taddr:%b time:%d",jump_cond,PC,valid,br_taddr,$time);
			  end
  			if(valid_exe)
			begin
				inst_rptr=inst_rptr+3'd1;
				PC_rptr=inst_rptr;
			end
			end

		end

         
endmodule


module Branch_target_buffer(rst,rd_sig,wr_sig,buffer_select,br_inst_read,br_inst_write,br_taddr,br_taddr_exe,valid);

	input rd_sig,wr_sig,buffer_select,rst;
	input [23:0]br_inst_read,br_inst_write;//7 bits of instruction is opcode
	input [31:0]br_taddr_exe;
  	reg [46:0]memory[1023:0];

	output reg [31:0]br_taddr;
 	output reg valid;


	always@(*)
	begin
	    if(rst)
	    begin
	//	for(i=0;i<1023;i=i+1)
	//	begin
	//		memory[i]=47'd0;//initialize the memory to all zeros
	//	end
 		valid=1'b0;
		br_taddr=31'd0;		
            end
	    else if(buffer_select)
	     begin
		if(wr_sig)//write
		begin
         	 memory[br_inst_write[9:0]]={1'b1,br_inst_write[23:10],br_taddr_exe};//valid,tag,branch target address
  		 $display("write BTB memory:%b time:%d",memory[br_inst_write[9:0]],$time);   		       
		end

		if(rd_sig)//read
		begin
		if(memory[br_inst_read[9:0]][45:32]==br_inst_read[23:10])//tag comparison
			begin
		
		 	br_taddr=memory[br_inst_read[9:0]][31:0];
		        valid=memory[br_inst_read[9:0]][46];
            end
			else//cache miss
			begin
	
			valid=1'b0;			
			br_taddr=32'd0;
			end
			$display("read BTB memory:%d tag:%d br_addr:%d valid:%d time:%d",memory[br_inst_read[9:0]][45:32],br_inst_read[23:10],br_taddr,valid,$time);
 		 end
		end
		else
		begin
			valid=1'b0;
		end
	     end



endmodule




module Branch_predictor(rst,LHT_index_read,LHT_index_write,rd_sig,write_sig,TAKEN_BRANCH,prediction_valid_exe,BP_select);

	input rst,rd_sig,write_sig,prediction_valid_exe,BP_select;
	input [3:0]LHT_index_read,LHT_index_write;
 	reg [3:0]LHT[15:0];
 	reg [1:0]LPT[15:0];
  	reg [3:0]LPT_index_read,LPT_index_write;
	output reg TAKEN_BRANCH;
	reg [1:0]Branch_pred;
  	reg [4:0]i;

  always@(*)
	begin
      	if(rst)
        begin
		for(i=0;i<16;i=i+1)
            begin
		    //$display("%d",i);
        	    LHT[i[3:0]]=4'd0;
              	    LPT[i[3:0]]=2'd0;
            end
          
        end
      	else if(BP_select)
        begin

		if(rd_sig)//read
		begin	
			LPT_index_read=LHT[LHT_index_read];
			Branch_pred=LPT[LPT_index_read];//2 bit TAKEN_BRANCH
		$display("lpt_index%d branch_pr:%d time:%d",LPT_index_read,LPT[LPT_index_read],$time);

			case (Branch_pred)//make 2 bit to 1 bit TAKEN_BRANCH
				2'b00:TAKEN_BRANCH=1'b0;
				2'b01:TAKEN_BRANCH=1'b0;
				2'b10:TAKEN_BRANCH=1'b1;
				2'b11:TAKEN_BRANCH=1'b1;
//				default:TAKEN_BRANCH=1'b0;
			endcase
		
		end
  		if(write_sig)//write
		begin
      			if(prediction_valid_exe)//if branch taken
			begin
				LPT_index_write=LHT[LHT_index_write];
				if(LPT[LPT_index_write]!=2'b11)
				LPT[LPT_index_write]=(LPT[LPT_index_write]+1);//update LPT\TAKEN_BRANCH
		$write("lp_index:%d lpt[]:%d time:%d",LPT_index_write,LPT[LPT_index_write],$time);
				LHT[LHT_index_write]={1'b1,LPT_index_write[3:1]};//update LHT
		$display("   updated lht:%d",LHT[LHT_index_write]);
			end
      			else if(!prediction_valid_exe)
			begin
			LPT_index_write=LHT[LHT_index_write];
			if(LPT[LPT_index_write]!=2'b00)
			LPT[LPT_index_write]=(LPT[LPT_index_write]-1);//update LPT
			$write("lp_index:%d lpt[]:%d time:%d",LPT_index_write,LPT[LPT_index_write],$time);
			LHT[LHT_index_write]={1'b0,LPT_index_write[3:1]};//update LHT
			$display("   updated lht:%d",LHT[LHT_index_write]);

			end
		end
        end
	end
 
endmodule
module Fetch_top(RESET,clk,branch_status_exe,valid_exe,jump_addr_exe,IF_ID_IR,PC,TAKEN_BRANCH,STALL,HALT,winc,wclk,PROG_INST);
input RESET,clk,branch_status_exe,valid_exe,winc,wclk;
input [31:0]jump_addr_exe,PROG_INST;
output wire[31:0]IF_ID_IR;
output wire [31:0]PC;
output TAKEN_BRANCH,STALL,HALT;
wire read_sig,write_sig,buffer_select,valid,branch_predictor_select,prediction_valid_exe;
wire [23:0]br_inst_read,br_inst_write;
wire [31:0]br_taddr,br_taddr_exe,instruction;
wire [3:0]LHT_index_read,LHT_index_write;


Branch_target_buffer BTB_1(.rst(RESET),.rd_sig(read_sig),.wr_sig(write_sig),.buffer_select(buffer_select),.br_inst_read(br_inst_read),.br_inst_write(br_inst_write),.br_taddr(br_taddr),.br_taddr_exe(br_taddr_exe),.valid(valid));

Branch_predictor BP(.rst(RESET),.LHT_index_read(LHT_index_read),.LHT_index_write(LHT_index_write),.rd_sig(read_sig),.write_sig(write_sig),.TAKEN_BRANCH(TAKEN_BRANCH),.prediction_valid_exe(prediction_valid_exe),.BP_select(branch_predictor_select));

Program_counter_control PC_control(.clk(clk),.branch_status_exe(branch_status_exe),.valid_exe(valid_exe),.valid(valid),.TAKEN_BRANCH(TAKEN_BRANCH),.rst(RESET),.jump_addr_exe(jump_addr_exe),.br_taddr(br_taddr),.br_inst_read(br_inst_read),.br_inst_write(br_inst_write),.read_sig(read_sig),.write_sig(write_sig),.buffer_select(buffer_select),.branch_predictor_select(branch_predictor_select),.STALL(STALL),.prediction_valid_exe(prediction_valid_exe),.LHT_index_read(LHT_index_read),.LHT_index_write(LHT_index_write),.PC(PC),.br_taddr_exe(br_taddr_exe),.IF_ID_IR(IF_ID_IR),.HALT(HALT),.instruction(instruction));

asynchronous_inst_mem prog_mem(.winc(winc),.wclk(wclk),.rclk(clk),.rst(RESET),.read_addr(PC),.wdata(PROG_INST),.rdata(instruction));
endmodule


module rv32de(IF_ID_IR,clk,PC_IN_DECODE,PC_OUT_DECODE,ID_EX_A,ID_EX_B,ID_EX_IR,ID_EX_type,read_sig,ID_EX_RD,STALL,HALT,ID_EX_Imm);
  input clk,STALL,HALT; 
  input [31:0]IF_ID_IR,PC_IN_DECODE;
  output reg[4:0]ID_EX_A,ID_EX_B,ID_EX_RD;
  output reg [31:0]ID_EX_IR,PC_OUT_DECODE,ID_EX_Imm;
  output reg [6:0]ID_EX_type;
  output reg read_sig;

  parameter I=19;//0010011
  parameter L=3;//0000011
  parameter S=35;//0100011 
  parameter LUI=55;//0110111 
  parameter AUIPC=23;//0010111 
  parameter B=99;//1100011
  parameter JAL=111;//1101111
  parameter JALR=103;//1100111
  
  always @(posedge clk)
    begin
	if(STALL)
	begin
	   ID_EX_type<=7'd0;
	   read_sig<=1'b0;
	   ID_EX_IR<=32'd0;
	end
	else
	begin
		if(!HALT)
		begin

      PC_OUT_DECODE<=PC_IN_DECODE;
      ID_EX_IR <= IF_ID_IR;
      ID_EX_type <= IF_ID_IR[6:0];
      ID_EX_A<=IF_ID_IR[19:15];
      ID_EX_B<=IF_ID_IR[24:20];
      ID_EX_RD<=IF_ID_IR[11:7];
      read_sig<=1'b1;
       case (IF_ID_IR[6:0]) 
	       I:
	       begin
		       ID_EX_Imm <= {{21{IF_ID_IR[31]}},IF_ID_IR[30:20]};
	       end
	       L:
	       begin
		       ID_EX_Imm <={{21{IF_ID_IR[31]}},IF_ID_IR[30:20]};

	       end
	       S:
	       begin
		       ID_EX_Imm <= {{21{IF_ID_IR[31]}},IF_ID_IR[31:25],IF_ID_IR[11:7]};
	       end
	       LUI:
	       begin
		       ID_EX_Imm<={IF_ID_IR[31:12],12'd0};
	       end
	       AUIPC:
	       begin
		       ID_EX_Imm<={IF_ID_IR[31:12],12'd0};
	       end
	       B:
	       begin
		        ID_EX_Imm <= {{21{IF_ID_IR[31]}},IF_ID_IR[7],IF_ID_IR[30:25],IF_ID_IR[11:8]};

	       end
	       JAL:
	       begin
			ID_EX_Imm <=  {{12{IF_ID_IR[31]}},IF_ID_IR[19:12],IF_ID_IR[20],IF_ID_IR[30:21],1'b0};

	       end
	       JALR:
	       begin
			ID_EX_Imm<= {{21{IF_ID_IR[31]}},IF_ID_IR[30:20]};
	       end
	       default:
		       ID_EX_Imm<=ID_EX_Imm;
       endcase	

	   end
      end
   end
endmodule

 module regbank(rd_data1, rd_data2, wr_data,rs1,rs2,rd_wb, write_sig,read_sig,EX_MEM_ALUOut,EX_MEM_rd,MEM_WB_ALUOut,MEM_WB_rd);
  input  write_sig,read_sig;
  input [4:0] rs1, rs2, rd_wb,EX_MEM_rd,MEM_WB_rd;
  input [31:0] wr_data,EX_MEM_ALUOut,MEM_WB_ALUOut;
  output reg [31:0] rd_data1, rd_data2;
  reg signed[31:0] regfile[0:31];


  always @(*)
    begin
      if (write_sig)
      begin
	      regfile[rd_wb] = wr_data;
      end
      if(read_sig)
        begin


         if(rs1==0) rd_data1 =0;
          else 
	  begin
		  if(EX_MEM_rd==rs1)
		  begin
			  rd_data1=EX_MEM_ALUOut;

		  end
		  else if(MEM_WB_rd==rs1)
			  rd_data1=MEM_WB_ALUOut;
		  else
			  rd_data1=regfile[rs1];

	  end
          if(rs2==0) rd_data2 =0;
          else
	  begin
		  if(EX_MEM_rd==rs2)
		  begin
			  rd_data2=EX_MEM_ALUOut;
		  end
		  else if(MEM_WB_rd==rs2)
			  rd_data2=MEM_WB_ALUOut;
		  else
			  rd_data2=regfile[rs2];

   end	
end
 end   
endmodule

module rv32ex(clk,ID_EX_type,ID_EX_IR,rs1,rs2,PC_IN_EXECUTE,PC_OUT_EXECUTE,VALID,BRANCH_STATUS,EX_MEM_ALUOut,EX_MEM_B,EX_MEM_IR,STALL,EX_MEM_type,HALT,ID_EX_RD,EX_MEM_RD,ID_EX_Imm);
  
  input [6:0]ID_EX_type;
  input clk,STALL,HALT;
  input [4:0]ID_EX_RD;
  input signed [31:0]rs1,rs2;
  input [31:0]ID_EX_IR;
  input [31:0]PC_IN_EXECUTE,ID_EX_Imm;
  output reg [31:0]EX_MEM_ALUOut,EX_MEM_IR,EX_MEM_B;
  output [31:0]PC_OUT_EXECUTE;
  output reg VALID,BRANCH_STATUS;
  output reg [6:0]EX_MEM_type;
  output reg [4:0]EX_MEM_RD;
  reg [31:0]mul_l,mul_h;
//  reg [31:0]ID_EX_Imm;
  reg [31:0]tPC;
  reg signed[31:0]ID_EX_A,ID_EX_B;
  reg [31:0]id_ex_a,id_ex_b;
  wire overflow;
  
  //opcodes and types of instructions
  parameter R=51;//0110011
  parameter I=19;//0010011
  parameter L=3;//0000011
  parameter S=35;//0100011 
  parameter LUI=55;//0110111 
  parameter AUIPC=23;//0010111 
  parameter B=99;//1100011
  parameter JAL=111;//1101111
  parameter JALR=103;//1100111

  //r type-0110011
  parameter ADD=0;
  parameter SLL=1;
  parameter SLT=2;
  parameter SLTU=3;
  parameter XOR=4;
  parameter SRL=5;
  parameter OR=6;
  parameter AND=7;
  parameter SUB=256;//as suggested by karthik-concatenation
  parameter SRA=261;
  parameter MUL=8;
  parameter MULH=9;
  parameter MULHSU=10;
  parameter MULHU=11;
  parameter DIV=12;
  parameter DIVU=13;
  parameter REM=14;
  parameter REMU=15;

  //i type -0010011
  parameter ADDI=0;
  parameter SLTI=2;
  parameter SLTIU=3;
  parameter XORI=4;
  parameter ORI=6;
  parameter ANDI=7;
  parameter SLLI=1;
  parameter SRLI=5;
  parameter SRAI=5;//NEED HIGHER IMMEDIATE BITS TO CHOOSE BETWEEN SRAI & SRLI

  //load -0000011
  parameter LB=0;
  parameter LH=1;
  parameter LW=2;
  parameter LBU=3;
  parameter LHU=5;

  //s-type-0100011
  parameter SB=0;
  parameter SH=1;
  parameter SW=2;


  //b type-1100011
  parameter BEQ=0;
  parameter BNE=1;
  parameter BLT=4;
  parameter BGE=5;
  parameter BLTU=6;
  parameter BGEU=7;
  always@(*)
  begin
	  if(STALL)
	  begin
  		  ID_EX_A=32'd0;
		  ID_EX_B=32'd0;
	 	  id_ex_a=32'd0;
		  id_ex_b=32'd0;
	  end
	  else
	  begin
      ID_EX_A=rs1;
      ID_EX_B=rs2;
      id_ex_a=rs1;
      id_ex_b=rs2;

	  end
  end
  always @(posedge clk)
  begin
	  if(STALL)
	  begin


		  EX_MEM_IR<=32'd0;
		  EX_MEM_type<=7'd0;
		  VALID<=1'b0;

	  end
	  else
    begin
 
     if(!HALT)
     begin

      EX_MEM_IR<=ID_EX_IR;
      EX_MEM_type<=ID_EX_type;
      case (ID_EX_type) 

        R:
          begin
	  EX_MEM_RD<=ID_EX_RD;

            case ({ID_EX_IR[31:25],ID_EX_IR[14:12]})
              ADD: EX_MEM_ALUOut <=  ID_EX_A + ID_EX_B;
              SUB: EX_MEM_ALUOut <=  ID_EX_A - ID_EX_B;
              AND: EX_MEM_ALUOut <=  ID_EX_A & ID_EX_B;
              OR:  EX_MEM_ALUOut <=  ID_EX_A | ID_EX_B;
               SLT: EX_MEM_ALUOut <= ID_EX_A < ID_EX_B;
              SRA: EX_MEM_ALUOut <=  ID_EX_A >>> ID_EX_B;
              SRL: EX_MEM_ALUOut <=  ID_EX_A >> ID_EX_B;
              SLL: EX_MEM_ALUOut <=  ID_EX_A << ID_EX_B;
              XOR: EX_MEM_ALUOut <=  ID_EX_A ^ ID_EX_B;

              SLTU: 
                begin
                  EX_MEM_ALUOut =  id_ex_a < id_ex_b;
                end

                  MUL:
                    begin
                      {mul_h,mul_l}=ID_EX_A*ID_EX_B;
                     EX_MEM_ALUOut = mul_l;
                    end

                  MULH:
                    begin
                      {mul_h,mul_l}=ID_EX_A*ID_EX_B;
                      EX_MEM_ALUOut= mul_h;
                    end

                  MULHU:
                    begin
                      {mul_h,mul_l}=id_ex_a * id_ex_b;
                      EX_MEM_ALUOut= mul_h;
                    end

                  MULHSU:
                    begin
                      {mul_h,mul_l}=ID_EX_A * id_ex_b;
                      EX_MEM_ALUOut= mul_h;
                    end

                  DIV:
                    begin
                      EX_MEM_ALUOut<=  ID_EX_A / ID_EX_B;
                    end

                  DIVU: 
                    begin
                      EX_MEM_ALUOut<=  id_ex_a / id_ex_b;
                    end

                  REM:
                    begin
                      EX_MEM_ALUOut<=  ID_EX_A % ID_EX_B;
                    end

                  REMU: 
                    begin
                      EX_MEM_ALUOut<= id_ex_a % id_ex_b;
                    end
		  default:
		  begin
			  EX_MEM_ALUOut<=32'd0;
		  end
                endcase
		VALID<=1'b0;
      BRANCH_STATUS<=1'b0;
          end

        I:
          begin
		  		  EX_MEM_RD<=ID_EX_RD;
  //          ID_EX_Imm <= {{21{ID_EX_IR[31]}},ID_EX_IR[30:20]};

            case (ID_EX_IR[14:12])
              ADDI: EX_MEM_ALUOut <=  ID_EX_A + ID_EX_Imm ;
              SLTI: EX_MEM_ALUOut <=  ID_EX_A < $signed(ID_EX_Imm );
              SLTIU:
                begin
          //        if(ID_EX_A<0)
           //         ID_EX_A <= ~ID_EX_A+1;
            //      if({{21{ID_EX_IR[31]}},ID_EX_IR[30:20]}<0)
             //       ID_EX_Imm <= ~{{21{ID_EX_IR[31]}},ID_EX_IR[30:20]}+1;
                  EX_MEM_ALUOut <=  id_ex_a < ID_EX_Imm ;
                end

              XORI:  EX_MEM_ALUOut <=  ID_EX_A ^ ID_EX_Imm ;
              ORI:   EX_MEM_ALUOut <=  ID_EX_A | ID_EX_Imm ;
              ANDI:  EX_MEM_ALUOut <=  ID_EX_A & ID_EX_Imm ;
              SLLI:  EX_MEM_ALUOut <=  ID_EX_A <<ID_EX_IR[25:20];
              5: 
                begin
                  if(ID_EX_IR[30]==1'b0)
                    EX_MEM_ALUOut <=  ID_EX_A >> ID_EX_IR[25:20];//SRLI
                  else
                    EX_MEM_ALUOut <=  ID_EX_A >>> ID_EX_IR[25:20];//SRAI
                end
		  default:
		  begin
			  EX_MEM_ALUOut<=32'd0;
		  end

            endcase
	          VALID<=1'b0;
      BRANCH_STATUS<=1'b0;
          end

        L://also I type
          begin
		    EX_MEM_RD<=ID_EX_RD;

    //        ID_EX_Imm <={{21{ID_EX_IR[31]}},ID_EX_IR[30:20]};
            case (ID_EX_IR[14:12])
              LB:EX_MEM_ALUOut <= ID_EX_A + ID_EX_Imm ;
              LH:EX_MEM_ALUOut <=  ID_EX_A +ID_EX_Imm ;
              LW:EX_MEM_ALUOut <=  ID_EX_A +ID_EX_Imm ;
              LBU:EX_MEM_ALUOut <= ID_EX_A +ID_EX_Imm ;
              LHU:EX_MEM_ALUOut <= ID_EX_A + ID_EX_Imm ;
	      		  default:
		  begin
			  EX_MEM_ALUOut<=32'd0;
		  end

            endcase
	     EX_MEM_B<=ID_EX_B;
	           VALID<=1'b0;
      BRANCH_STATUS<=1'b0;
          end

        S:

          begin
		  EX_MEM_RD<=5'd0;
      //      ID_EX_Imm <= {{21{ID_EX_IR[31]}},ID_EX_IR[31:25],ID_EX_IR[11:7]};
            case (ID_EX_IR[14:12])
              SB:EX_MEM_ALUOut <=  ID_EX_A + ID_EX_Imm ;
              SH:EX_MEM_ALUOut <=  ID_EX_A + ID_EX_Imm ;
              SW:EX_MEM_ALUOut <=  ID_EX_A + ID_EX_Imm ;
	      		  default:
		  begin
			  EX_MEM_ALUOut<=32'd0;
		  end

            endcase
	     EX_MEM_B<=ID_EX_B;
	           VALID<=1'b0;
      BRANCH_STATUS<=1'b0;
          end


 
        LUI:
          begin
           EX_MEM_ALUOut<= ID_EX_Imm ;
           	          VALID<=1'b0;
      BRANCH_STATUS<=1'b0;
      		    EX_MEM_RD<=ID_EX_RD;
          end

        AUIPC:
          begin
		    EX_MEM_RD<=ID_EX_RD;

           EX_MEM_ALUOut<= ID_EX_Imm +PC_IN_EXECUTE;
              VALID<=1'b0;
      BRANCH_STATUS<=1'b0;
          end

        B:
          begin
            		    EX_MEM_RD<=5'd0;

        //    ID_EX_Imm <= {{21{ID_EX_IR[31]}},ID_EX_IR[7],ID_EX_IR[30:25],ID_EX_IR[11:8]};
            case (ID_EX_IR[14:12])
              BEQ:
		      begin
			if(ID_EX_A == ID_EX_B)
      		  	begin
                	tPC<= PC_IN_EXECUTE+ID_EX_Imm ;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
			$display("BEQ %d",$time);
		  	end
			else
			begin
				VALID<=1'b1;
				BRANCH_STATUS<=1'b0;
			end
		end
              BNE:
		      begin
			      if(ID_EX_A != ID_EX_B)
      		  begin
                	tPC<= PC_IN_EXECUTE+ID_EX_Imm ;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
			$display("BNE %d",$time);

		  end
 			else
			begin
				VALID<=1'b1;
				BRANCH_STATUS<=1'b0;
			end

	 	 end
              BLT:
		      begin
			      if(ID_EX_A <= ID_EX_B)
      		  begin
                	tPC<= PC_IN_EXECUTE+ID_EX_Imm ;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
			$display("BLT%d",$time);

		  end
		  			else
			begin
				VALID<=1'b1;
				BRANCH_STATUS<=1'b0;
			end

	  	end
              BGE:
		      begin
			      if(ID_EX_A >= ID_EX_B)
      		  begin
                	tPC<= PC_IN_EXECUTE+ID_EX_Imm ;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
			$display("BGE%d",$time);

		  end
		else
			begin
				VALID<=1'b1;
				BRANCH_STATUS<=1'b0;
			end

	  	end
              BLTU:
	      
                begin

                  if( id_ex_a<= id_ex_b)
      		  begin

                	tPC<= PC_IN_EXECUTE+ID_EX_Imm ;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
			$display("BLTU a %d b %d %d",id_ex_a,id_ex_b,$time);

		  end
			else
			begin
				VALID<=1'b1;
				BRANCH_STATUS<=1'b0;
			end

                end

              BGEU:
                begin


                  if(id_ex_a >= id_ex_b)
      		  begin
                	tPC<= PC_IN_EXECUTE+ID_EX_Imm ;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;


		  end
		else
		begin
				VALID<=1'b1;
				BRANCH_STATUS<=1'b0;
		end

		 	$display("BGEU a %d b %d %d",id_ex_a,id_ex_b,$time);
                end
	  default:
		  begin
			  VALID<=1'b0;
		  end


            endcase
          end

        JAL:
          begin
		  		    EX_MEM_RD<=ID_EX_RD;

          //  ID_EX_Imm <=  {{12{ID_EX_IR[31]}},ID_EX_IR[19:12],ID_EX_IR[20],ID_EX_IR[30:21],1'b0};
            EX_MEM_ALUOut <= PC_IN_EXECUTE+1;
            tPC<=PC_IN_EXECUTE+ID_EX_Imm ;
	    VALID<=1'b1;
	    BRANCH_STATUS<=1'b1;
        	$display("JAL %d",$time);

          end

        JALR:
          begin
		  		    EX_MEM_RD<=ID_EX_RD;

            //ID_EX_Imm<= {{21{ID_EX_IR[31]}},ID_EX_IR[30:20]};
            EX_MEM_ALUOut <= PC_IN_EXECUTE+1;
            tPC<=ID_EX_A+ID_EX_Imm ;
	    VALID<=1'b1;
	    BRANCH_STATUS<=1'b1;
			$display("JALR %d",$time);
          end
	default:
	begin
		VALID<=1'b0;
	end
      endcase
	end
    end
    end
  assign PC_OUT_EXECUTE=tPC;
  assign overflow = (EX_MEM_ALUOut[31] & ~ID_EX_A[31] & ~ID_EX_B[31])||(~EX_MEM_ALUOut[31] & ID_EX_A[31] & ID_EX_B[31] );
endmodule 

module rv32_mem(EX_MEM_type,EX_MEM_B,clk,MEM_WB_ALUOut,EX_MEM_ALUOut,MEM_WB_IR,EX_MEM_IR,STALL,HALT,EX_MEM_RD,MEM_WB_RD);
  parameter load=3;
  parameter register=51;
  parameter immediate=19;
  parameter store=35; 
  
  input clk,STALL,HALT;
  input [4:0]EX_MEM_RD;
  input [6:0]EX_MEM_type;
  input [31:0] EX_MEM_B,EX_MEM_ALUOut,EX_MEM_IR;
  output reg [31:0] MEM_WB_ALUOut,MEM_WB_IR;
  output reg [4:0]MEM_WB_RD;
  
  reg [31:0] Data_Memory [0:1023]; // Data memory
  always @(posedge clk)
    begin

	   if((!STALL)&&(!HALT))
	    begin
		    MEM_WB_RD<=EX_MEM_RD;
      case(EX_MEM_type)
        load:
          MEM_WB_ALUOut <=  Data_Memory[EX_MEM_ALUOut];
        store:
          Data_Memory[EX_MEM_ALUOut] <=  EX_MEM_B;
        default:
          MEM_WB_ALUOut <=  EX_MEM_ALUOut;
      endcase
      MEM_WB_IR<=EX_MEM_IR;
    end
    end
endmodule

module rv32_wb(MEM_WB_ALUOut,clk,MEM_WB_IR,write_sig,address,data,HALT);

  input [31:0] MEM_WB_ALUOut;
  input [31:0] MEM_WB_IR;
  input clk,HALT;
  output reg write_sig;
  output reg [4:0]address;
  output reg [31:0]data;
  always@(posedge clk)
  begin
	  if(!HALT)
	  begin
	  address<=MEM_WB_IR[11:7];
	  data<=MEM_WB_ALUOut;
	  if((MEM_WB_IR[6:0]==7'd35)||(MEM_WB_IR[6:0]==7'd99))
		  write_sig<=1'b0;
	  else
	  write_sig<=1'b1;
  	  end
  end
 
endmodule
module rv32im_top (RESET,clk,wclk,winc,PROG_INST,MEM_WB_IR);
  input RESET,clk,wclk,winc;
  input [31:0]PROG_INST;
  wire [31:0]IF_ID_IR,ID_EX_IR,PC,rs1,rs2,PC_OUT_DECODE,PC_OUT_EXECUTE,EX_MEM_B,MEM_WB_ALUOut,EX_MEM_ALUOut,EX_MEM_IR,data,rd_data1,rd_data2,ID_EX_Imm;

  wire [6:0]ID_EX_type,EX_MEM_type;
  wire TAKEN_BRANCH,STALL,BRANCH_STATUS,VALID,read_sig,write_sig;
  wire HALT;
  wire [4:0]ID_EX_A,ID_EX_B,ID_EX_RD,address,EX_MEM_RD,MEM_WB_RD;
 // output test;
 // assign test=1'b1;
 output [31:0]MEM_WB_IR;
  reg [31:0]PC_reg;


  always@(posedge clk)
  begin
	if(!RESET)
	begin
	        PC_reg<=PC;
  	end
  end
  Fetch_top fetch(.RESET(RESET),.clk(clk),.branch_status_exe(BRANCH_STATUS),.valid_exe(VALID),.jump_addr_exe(PC_OUT_EXECUTE),.IF_ID_IR(IF_ID_IR),.PC(PC),.TAKEN_BRANCH(TAKEN_BRANCH),.STALL(STALL),.HALT(HALT),.winc(winc),.wclk(wclk),.PROG_INST(PROG_INST));

  rv32de decode(.IF_ID_IR(IF_ID_IR),.clk(clk),.PC_IN_DECODE(PC_reg),.PC_OUT_DECODE(PC_OUT_DECODE),.ID_EX_A(ID_EX_A),.ID_EX_B(ID_EX_B),.ID_EX_IR(ID_EX_IR),.ID_EX_type(ID_EX_type),.read_sig(read_sig),.ID_EX_RD(ID_EX_RD),.STALL(STALL),.HALT(HALT),.ID_EX_Imm(ID_EX_Imm ));

  regbank register_bank(.rd_data1(rd_data1),.rd_data2(rd_data2),.wr_data(data),.rs1(ID_EX_A),.rs2(ID_EX_B),.rd_wb(address),.write_sig(write_sig),.read_sig(read_sig),.EX_MEM_ALUOut(EX_MEM_ALUOut),.EX_MEM_rd(EX_MEM_RD),.MEM_WB_ALUOut(MEM_WB_ALUOut),.MEM_WB_rd(MEM_WB_RD));
 
  rv32ex execute_stage(.clk(clk),.ID_EX_type(ID_EX_type),.ID_EX_IR(ID_EX_IR),.rs1(rd_data1),.rs2(rd_data2),.PC_IN_EXECUTE(PC_OUT_DECODE),.PC_OUT_EXECUTE(PC_OUT_EXECUTE),.VALID(VALID),.BRANCH_STATUS(BRANCH_STATUS),.EX_MEM_ALUOut(EX_MEM_ALUOut),.EX_MEM_B(EX_MEM_B),.EX_MEM_IR(EX_MEM_IR),.STALL(STALL),.EX_MEM_type(EX_MEM_type),.HALT(HALT),.ID_EX_RD(ID_EX_RD),.EX_MEM_RD(EX_MEM_RD),.ID_EX_Imm(ID_EX_Imm ));

  rv32_mem memory_stage(.EX_MEM_type(EX_MEM_type),.EX_MEM_B(EX_MEM_B),.clk(clk),.MEM_WB_ALUOut(MEM_WB_ALUOut),.EX_MEM_ALUOut(EX_MEM_ALUOut),.MEM_WB_IR(MEM_WB_IR),.EX_MEM_IR(EX_MEM_IR),.STALL(STALL),.HALT(HALT),.EX_MEM_RD(EX_MEM_RD),.MEM_WB_RD(MEM_WB_RD));

  rv32_wb write_back(.MEM_WB_ALUOut(MEM_WB_ALUOut),.clk(clk),.MEM_WB_IR(MEM_WB_IR),.write_sig(write_sig),.address(address),.data(data),.HALT(HALT));
endmodule
