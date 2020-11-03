
module Program_counter_control(clk,branch_status_exe,valid_exe,valid,TAKEN_BRANCH,rst,jump_addr_exe,br_taddr,br_inst,read_write,buffer_select,branch_predictor_select,STALL,prediction_valid_exe,LHT_index,PC,br_taddr_exe,IF_ID_IR);
	parameter JAL=7'b1101111;
	parameter JALR=7'b1100111;
	parameter B_inst=7'b1100011;
//	input [31:0]instruction;
	input clk,branch_status_exe,valid_exe,valid,TAKEN_BRANCH,rst;
        input [31:0]jump_addr_exe,br_taddr;
	output reg [23:0]br_inst;
	output reg read_write,buffer_select,branch_predictor_select,STALL,prediction_valid_exe;
        output reg [3:0]LHT_index;
	output reg [31:0]PC,br_taddr_exe,IF_ID_IR;
	reg [2:0]inst_rptr,inst_wptr,PC_rptr,PC_wptr;
  	reg [1:0]jump_cond;
  	reg [24:0]instruction_fifo[7:0];
	reg [31:0]PC_fifo[7:0];
	reg [31:0]Mem[1023:0];//instruction memory
	reg PC_jump_halt;
	always@(*)
	begin


      			if(valid_exe)//valid data from execution stage-branch target address and jump status
			begin
				if(branch_status_exe!=instruction_fifo[inst_rptr][0])//branch mispredicted
				 STALL=1'b1;//
            			br_inst=(instruction_fifo[inst_rptr][24:1]);
				read_write=1'b0;//write
				buffer_select=1'b1;//br_inst[0];//choose one of the two BTB
            			branch_predictor_select=1'b1;
				br_taddr_exe=jump_addr_exe;//from exe stage
				prediction_valid_exe=branch_status_exe;//from exe stage
            			LHT_index=br_inst[3:0];
				$display("exe stage data branch_status_exe :%d br_inst:%b read_write:%d buffer_select:%b br_tadddr_exe:%b prediction_valid_exe:%d LHT_index:%d time:%d",branch_status_exe,br_inst,read_write,buffer_select,branch_predictor_select,br_taddr_exe,prediction_valid_exe,LHT_index,$time);
			end
      			if(IF_ID_IR[6:0]==B_inst)//conditional jump
			begin
				jump_cond=2'b01;//conditional jump
				read_write=1'b1;//read
				buffer_select=1'b1;//instruction[7];
            			branch_predictor_select=1'b1;
				br_inst=IF_ID_IR[31:8];
            			LHT_index=IF_ID_IR[11:8];
				
                        	$display("conditional jump jump_cond:%d br_inst:%b read_write:%d buffer_select:%b branch_predictor_select:%d LHT_index:%d time:%d",jump_cond,br_inst,read_write,buffer_select,branch_predictor_select,LHT_index,$time);

			end
			else if((IF_ID_IR[6:0]==JAL)||(IF_ID_IR[6:0]==JALR))
			begin
				jump_cond=2'b10;//unconditional jump
				read_write=1'b1;//read
				buffer_select=1'b1;//instruction[7];
				br_inst=IF_ID_IR[31:8];
            			branch_predictor_select=1'b0;//Branch TAKEN_BRANCH not required for unconditional jump
				$display("unconditional jump jump_cond:%d br_inst:%b read_write:%d buffer_select:%b branch_predictor_select:%d LHT_index:%d time:%d",jump_cond,br_inst,read_write,buffer_select,branch_predictor_select,LHT_index,$time);

			end
			else
			begin
				jump_cond=2'b00;//no jump
				buffer_select=1'b0;
				branch_predictor_select=1'b0;
				$display("no jump state jump_cond:%d buffer_select:%d time:%d",jump_cond,buffer_select,$time);
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

			PC<=31'd0;
			PC_jump_halt<=1'b0;
		end
		else
		begin
			if(valid_exe)
				inst_rptr=inst_rptr+3'd1;
			if(!((PC_jump_halt==1'b0)&&(jump_cond!=2'b00)))//jump instruction encountered at previous clk cycle
			begin

		        	IF_ID_IR<=Mem[PC];

			end
			if(jump_cond!=2'b00)
				PC_jump_halt<=!PC_jump_halt;
			

			if((jump_cond==2'b10)||(jump_cond==2'b01))
			begin
				instruction_fifo[inst_wptr]<={IF_ID_IR[31:7],TAKEN_BRANCH};
				PC_fifo[PC_wptr]<=PC;
				inst_wptr<=inst_wptr+3'd1;
				PC_wptr<=PC_wptr+3'd1;
				$display("cache miss wptr%d time %d",inst_wptr,$time);
			end
			if((((PC_jump_halt==1'b0)&&(jump_cond!=2'b00))))
				PC<=PC;
			else
			begin
			if(STALL)//control hazard
			begin
        			  if(!(instruction_fifo[inst_rptr][0]))//branch mispredicted - not taken
					PC<=br_taddr_exe;
				  else
              				PC<=PC_fifo[PC_rptr];
				  PC_rptr=PC_rptr+3'd1;
				  STALL<=1'b0;
				  $display("Stall branch_predicted:%d PC:%b STALL:%d time:%d",instruction_fifo[inst_rptr][0],PC,STALL,$time);

			end
			else
			begin
				if(jump_cond==2'b00)
					PC<=PC+32'd1;
				else if(jump_cond==2'b10)
				begin
					if(valid)
						PC<=br_taddr;
					else
						PC<=PC+32'd1;
				end	
				else if(jump_cond==2'b01)
				begin
					 if(valid)
					 begin
						if(TAKEN_BRANCH)
							PC<=br_taddr;
					 end
	  				 else
						PC<=PC+32'd1;
            			 end
				 else
					PC<=PC+32'd1;
				 $display("PC updation jump_cond:%d PC:%b valid:%d br_taddr:%b time:%d",jump_cond,PC,valid,br_taddr,$time);

			end
		end

		end
         end

endmodule


module Branch_target_buffer(rst,rd_wr,buffer_select,br_inst,br_taddr,br_taddr_exe,valid);

	input rd_wr,buffer_select,rst;
	input [23:0]br_inst;//7 bits of instruction is opcode
	input [31:0]br_taddr_exe;
  	reg [46:0]memory[1023:0];

	output reg [31:0]br_taddr;
 	output reg valid;
  	reg [10:0]i;

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
		if(!rd_wr)//write
		begin
         	 memory[br_inst[9:0]]={1'b1,br_inst[23:10],br_taddr_exe};//valid,tag,branch target address
  		 $display("write BTB memory:%b time:%d",memory[br_inst[9:0]],$time);   		       
		end

		if(rd_wr)//read
		begin
		if(memory[br_inst[9:0]][45:32]==br_inst[23:10])//tag comparison
			begin
		
		 	br_taddr=memory[br_inst[9:0]][31:0];
		        valid=memory[br_inst[9:0]][46];
            end
			else//cache miss
			begin
	
			valid=1'b0;			
			br_taddr=32'd0;
			end
			$display("read BTB memory:%d tag:%d br_addr:%d valid:%d time:%d",memory[br_inst[9:0]][45:32],br_inst[23:10],br_taddr,valid,$time);
 		 end
		end
	     end



endmodule




module Branch_predictor(rst,LHT_index,rd_write,TAKEN_BRANCH,prediction_valid_exe,BP_select);

	input rst,rd_write,prediction_valid_exe,BP_select;
	input [3:0]LHT_index;
 	reg [3:0]LHT[31:0];
 	reg [1:0]LPT[15:0];
  	reg [3:0]LPT_index;
	output reg TAKEN_BRANCH;
	reg [2:0]Branch_pred;
  	reg [5:0]i;

  always@(*)
	begin
      	if(rst)
        begin
          for(i=0;i<32;i=i+1)
            begin
		    //$display("%d",i);
        	    LHT[i]=4'd0;
              	    LPT[i]=2'd0;
            end
          
        end
      	else if(BP_select)
        begin

		if(rd_write)//read
		begin	
			LPT_index=LHT[LHT_index];
			Branch_pred=LPT[LPT_index];//2 bit TAKEN_BRANCH
		$display("lpt_index%d branch_pr:%d time:%d",LPT_index,LPT[LPT_index],$time);

			case (Branch_pred)//make 2 bit to 1 bit TAKEN_BRANCH
				2'b00:TAKEN_BRANCH=1'b0;
				2'b01:TAKEN_BRANCH=1'b0;
				2'b10:TAKEN_BRANCH=1'b1;
				2'b11:TAKEN_BRANCH=1'b1;
				default:TAKEN_BRANCH=1'b0;
			endcase
		
		end
  		if(!rd_write)//write
		begin
      			if(prediction_valid_exe)//if branch taken
			begin
				LPT_index=LHT[LHT_index];
				LPT[LPT_index]=(LPT[LPT_index]+1)&&(LPT[LPT_index]!=2'b11);//update LPT\TAKEN_BRANCH
		$write("lp_index:%d lpt[]:%d time:%d",LPT_index,LPT[LPT_index],$time);
				LHT[LHT_index]={1'b1,LPT_index[3:1]};//update LHT
		$display("   updated lht:%d",LHT[LHT_index]);
			end
      			else if(!prediction_valid_exe)
			begin
			LPT_index=LHT[LHT_index];
			LPT[LPT_index]=(LPT[LPT_index]-1)&&(LPT[LPT_index]!=2'b00);//update LPT
			$write("lp_index:%d lpt[]:%d time:%d",LPT_index,LPT[LPT_index],$time);
			LHT[LHT_index]={1'b0,LPT_index[3:1]};//update LHT
			$display("   updated lht:%d",LHT[LHT_index]);

			end
		end
        end

 end
endmodule

module Fetch_top(RESET,clk,branch_status_exe,valid_exe,jump_addr_exe,IF_ID_IR,PC,TAKEN_BRANCH,STALL);
input RESET,clk,branch_status_exe,valid_exe;
input [31:0]jump_addr_exe;
output wire[31:0]IF_ID_IR;
output wire [31:0]PC;
output TAKEN_BRANCH,STALL;
wire read_write,buffer_select,valid,branch_predictor_select,prediction_valid_exe;
wire [23:0]br_inst;
wire [31:0]br_taddr,br_taddr_exe;
wire [3:0]LHT_index;


Branch_target_buffer BTB_1(.rst(RESET),.rd_wr(read_write),.buffer_select(buffer_select),.br_inst(br_inst),.br_taddr(br_taddr),.br_taddr_exe(br_taddr_exe),.valid(valid));

Branch_predictor BP(.rst(RESET),.LHT_index(LHT_index),.rd_write(read_write),.TAKEN_BRANCH(TAKEN_BRANCH),.prediction_valid_exe(prediction_valid_exe),.BP_select(branch_predictor_select));

Program_counter_control PC_control(.clk(clk),.branch_status_exe(branch_status_exe),.valid_exe(valid_exe),.valid(valid),.TAKEN_BRANCH(TAKEN_BRANCH),.rst(RESET),.jump_addr_exe(jump_addr_exe),.br_taddr(br_taddr),.br_inst(br_inst),.read_write(read_write),.buffer_select(buffer_select),.branch_predictor_select(branch_predictor_select),.STALL(STALL),.prediction_valid_exe(prediction_valid_exe),.LHT_index(LHT_index),.PC(PC),.br_taddr_exe(br_taddr_exe),.IF_ID_IR(IF_ID_IR));
endmodule


module rv32de(IF_ID_IR,clk,PC_IN_DECODE,PC_OUT_DECODE,ID_EX_A,ID_EX_B,ID_EX_IR,ID_EX_type,read_sig,ID_EX_RD);
  input clk; 
  input [31:0]IF_ID_IR,PC_IN_DECODE;
  output reg[4:0]ID_EX_A,ID_EX_B,ID_EX_RD;// using output reg here made it non synthesizable
  output reg [31:0]ID_EX_IR,PC_OUT_DECODE;
  output reg [6:0]ID_EX_type;
  output reg read_sig;

  always @(posedge clk)
    begin
      PC_OUT_DECODE<=PC_IN_DECODE;
      ID_EX_IR <= IF_ID_IR;
      ID_EX_type <= IF_ID_IR[6:0];
      ID_EX_A<=IF_ID_IR[19:15];
      ID_EX_B<=IF_ID_IR[24:20];
      ID_EX_RD<=IF_ID_IR[11:7];
      read_sig<=1'b1;
    end
endmodule

 module regbank(rd_data1, rd_data2, wr_data,rs1,rs2,rd_wb,rd_decode, write_sig,read_sig,clk,forward);
  input clk, write_sig,read_sig;
  input [4:0] rs1, rs2, rd_wb,rd_decode;
  input [31:0] wr_data;
  output reg [31:0] rd_data1, rd_data2;
  output reg [1:0]forward;
  reg [31:0] regfile[0:31];
  reg scoreboard[31:0];

  always @(posedge clk)
    begin
      if (write_sig)
      begin
	      regfile[rd_wb] <= wr_data;
	      if(scoreboard[rd_wb]==1'b1)
		      scoreboard[rd_wb]<=1'b0;
      end
      if(read_sig)
        begin

         if(rs1==0) rd_data1 <=0;
          else 
	  begin
		  rd_data1 <= regfile[rs1];
		  if(scoreboard[rs1]==1'b1)
			  forward<=2'b01;
		  else
			  forward<=2'b00;

	  end
          if(rs2==0) rd_data2 <=0;
          else
	  begin
		  rd_data2 <= regfile[rs2];
		  if(scoreboard[rs2]==1'b1)
			  forward<=2'b10;
		  else
			  forward<=2'b00;
	  end
	  scoreboard[rd_decode]<=1'b1;
        end
end	
    
endmodule

module rv32ex(forward,clk,ID_EX_type,ID_EX_IR,rs1,rs2,PC_IN_EXECUTE,PC_OUT_EXECUTE,VALID,BRANCH_STATUS,EX_MEM_ALUOut,EX_MEM_ALUOut_feedback,EX_MEM_B,EX_MEM_IR);
  
  input [6:0]ID_EX_type;
  input clk;
  input [1:0]forward;
  input signed [31:0]rs1,rs2;
  input [31:0]ID_EX_IR,EX_MEM_ALUOut_feedback;
  input [31:0]PC_IN_EXECUTE;
  output reg [31:0]EX_MEM_ALUOut,EX_MEM_IR,EX_MEM_B;
  output [31:0]PC_OUT_EXECUTE;
  output reg VALID,BRANCH_STATUS;
  reg [31:0]mul_l,mul_h;
  reg [31:0]ID_EX_Imm;
  reg [31:0]tPC;
  reg signed[31:0]ID_EX_A,ID_EX_B;
  
 
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


  always @(posedge clk)
    begin
      tPC<=PC_IN_EXECUTE;
      if(forward==2'b01)
	      ID_EX_A<=EX_MEM_ALUOut_feedback;
      if(forward==2'b10)
	      ID_EX_B<=EX_MEM_ALUOut_feedback;
      else 
      begin
      ID_EX_A<=rs1;
      ID_EX_B<=rs2;
      EX_MEM_IR<=ID_EX_IR;
      end
      case (ID_EX_type) 

        R:
          begin
            case ({ID_EX_IR[31:25],ID_EX_IR[14:12]})
              ADD: EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_B;
              SUB: EX_MEM_ALUOut <= #2 ID_EX_A - ID_EX_B;
              AND: EX_MEM_ALUOut <= #2 ID_EX_A & ID_EX_B;
              OR:  EX_MEM_ALUOut <= #2 ID_EX_A | ID_EX_B;
               SLT: EX_MEM_ALUOut <= #2 ID_EX_A < ID_EX_B;
              SRA: EX_MEM_ALUOut <= #2 ID_EX_A >>> ID_EX_B;
              SRL: EX_MEM_ALUOut <= #2 ID_EX_A >> ID_EX_B;
              SLL: EX_MEM_ALUOut <= #2 ID_EX_A << ID_EX_B;
              XOR: EX_MEM_ALUOut <= #2 ID_EX_A ^ ID_EX_B;

              SLTU: 
                begin
                  if(ID_EX_A<0)
                    ID_EX_A <= ~ID_EX_A+1;
                  if(ID_EX_B<0)
                    ID_EX_B <= ~ID_EX_B+1;
                  EX_MEM_ALUOut <= #2 ID_EX_A < ID_EX_B;
                end

              MUL:
                begin
                  mul_l<= #2 ID_EX_A*ID_EX_B;
                  EX_MEM_ALUOut<=mul_l;
                end

              MULH:
                begin
                  mul_h<= #2 ID_EX_A*ID_EX_B;
                  EX_MEM_ALUOut<=mul_h;
                end

              MULHU:
                begin
                  if(ID_EX_A<0)
                    ID_EX_A <= ~ID_EX_A+1;
                  if(ID_EX_B<0)
                    ID_EX_B <= ~ID_EX_B+1;
                  mul_h<= #2 ID_EX_A*ID_EX_B;
                  EX_MEM_ALUOut<=mul_h;
                end

              MULHSU:
                begin
                  if(ID_EX_B<0)
                    ID_EX_B <= ~ID_EX_B+1;
                  mul_h<= #2 ID_EX_A*ID_EX_B;
                  EX_MEM_ALUOut<=mul_h;
                end

              DIV:
                begin
                  mul_l<= #2 ID_EX_A / ID_EX_B;
                  EX_MEM_ALUOut<=mul_l;
                end

              DIVU: 
                begin
                  if(ID_EX_A<0)
                    ID_EX_A <= ~ID_EX_A+1;
                  if(ID_EX_B<0)
                    ID_EX_B <= ~ID_EX_B+1;
                  mul_l<= #2 ID_EX_A / ID_EX_B;
                  EX_MEM_ALUOut<=mul_l;
                end

              REM:
                begin
                  mul_l<= #2 ID_EX_A % ID_EX_B;
                  EX_MEM_ALUOut<= mul_l;
                end

              REMU: 
                begin
                  if(ID_EX_A<0)
                    ID_EX_A <= ~ID_EX_A+1;
                  if(ID_EX_B<0)
                    ID_EX_B <= ~ID_EX_B+1;
                  mul_l<= #2 ID_EX_A % ID_EX_B;
                  EX_MEM_ALUOut<= mul_l;
                end
            endcase
	          VALID=1'b0;
      BRANCH_STATUS=1'b0;
          end

        I:
          begin
            ID_EX_Imm <= ID_EX_IR[31:20];

            case (ID_EX_IR[14:12])
              ADDI: EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
              SLTI: EX_MEM_ALUOut <= #2 ID_EX_A < ID_EX_Imm;
              SLTIU:
                begin
                  if(ID_EX_A<0)
                    ID_EX_A <= ~ID_EX_A+1;
                  if(ID_EX_Imm<0)
                    ID_EX_Imm <= ~ID_EX_Imm+1;
                  EX_MEM_ALUOut <= #2 ID_EX_A < ID_EX_Imm;
                end

              XORI:  EX_MEM_ALUOut <= #2 ID_EX_A ^ ID_EX_Imm;
              ORI:   EX_MEM_ALUOut <= #2 ID_EX_A | ID_EX_Imm;
              ANDI:  EX_MEM_ALUOut <= #2 ID_EX_A & ID_EX_Imm;
              SLLI:  EX_MEM_ALUOut <= #2 ID_EX_A << ID_EX_Imm[5:0];
              5: 
                begin
                  if(ID_EX_IR[30]==1'b0)
                    EX_MEM_ALUOut <= #2 ID_EX_A >> ID_EX_Imm[5:0];//SRLI
                  else
                    EX_MEM_ALUOut <= #2 ID_EX_A >>> ID_EX_Imm[5:0];//SRAI
                end
            endcase
	          VALID=1'b0;
      BRANCH_STATUS=1'b0;
          end

        L://also I type
          begin
            ID_EX_Imm <= ID_EX_IR[31:20];
            case (ID_EX_IR[14:12])
              LB:EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
              LH:EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
              LW:EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
              LBU:EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
              LHU:EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
            endcase
	     EX_MEM_B<=ID_EX_B;
	           VALID=1'b0;
      BRANCH_STATUS=1'b0;
          end

        S:
          begin
            ID_EX_Imm <= ID_EX_IR[31:25]+ID_EX_IR[11:7];
            case (ID_EX_IR[14:12])
              SB:EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
              SH:EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
              SW:EX_MEM_ALUOut <= #2 ID_EX_A + ID_EX_Imm;
            endcase
	     EX_MEM_B<=ID_EX_B;
	           VALID=1'b0;
      BRANCH_STATUS=1'b0;
          end



        LUI:
          begin
            ID_EX_Imm <= ID_EX_IR[31:12];
            EX_MEM_ALUOut <= {ID_EX_Imm,12'b0};
	          VALID=1'b0;
      BRANCH_STATUS=1'b0;
          end

        AUIPC:
          begin
            ID_EX_Imm <= ID_EX_IR[31:12];
            EX_MEM_ALUOut<= tPC+{ID_EX_Imm,12'b0};
	          VALID=1'b0;
      BRANCH_STATUS=1'b0;
          end

        B:
          begin
            
            ID_EX_Imm <= {ID_EX_IR[31],ID_EX_IR[7],ID_EX_IR[30:25],ID_EX_IR[11:6],1'b0};
            case (ID_EX_IR[14:12])
              BEQ:if(ID_EX_A == ID_EX_B)
      		  begin
                	tPC<= tPC+ID_EX_Imm;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
		  end
              BNE:if(ID_EX_A != ID_EX_B)
      		  begin
                	tPC<= tPC+ID_EX_Imm;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
		  end
              BLT:if(ID_EX_A < ID_EX_B)
      		  begin
                	tPC<= tPC+ID_EX_Imm;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
		  end
              BGE:if(ID_EX_A > ID_EX_B)
      		  begin
                	tPC<= tPC+ID_EX_Imm;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
		  end
              BLTU:
                begin
                  if(ID_EX_A<0)
		  begin
                    ID_EX_A <= ~ID_EX_A+1;
		    VALID<=1'b1;
		    BRANCH_STATUS<=1'b1;
	    	  end
                  if(ID_EX_B<0)
		  begin
                    ID_EX_B <= ~ID_EX_B+1;
		    VALID<=1'b1;
		    BRANCH_STATUS<=1'b1;
		  end
                  if(ID_EX_A < ID_EX_B)
      		  begin
                	tPC<= tPC+ID_EX_Imm;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
		  end
                end

              BGEU:
                begin
                  if(ID_EX_A<0)
		  begin
                    ID_EX_A <= ~ID_EX_A+1;
		    VALID<=1'b1;
		    BRANCH_STATUS<=1'b1;
	    	  end
                  if(ID_EX_B<0)
		  begin
                    ID_EX_B <= ~ID_EX_B+1;
		    VALID<=1'b1;
		    BRANCH_STATUS<=1'b1;
		  end
                  if(ID_EX_A > ID_EX_B)
      		  begin
                	tPC<= tPC+ID_EX_Imm;
			VALID<=1'b1;
			BRANCH_STATUS<=1'b1;
		  end
                end
            endcase
          end

        JAL:
          begin
            ID_EX_Imm <=  {ID_EX_IR[31],ID_EX_IR[19:12],ID_EX_IR[20],ID_EX_IR[30:21],1'b0};
            EX_MEM_ALUOut <= tPC+1;
            tPC=tPC+ID_EX_Imm;
	    VALID<=1'b1;
	    BRANCH_STATUS<=1'b1;
          end

        JALR:
          begin
            ID_EX_Imm<= ID_EX_IR[31:20];
            EX_MEM_ALUOut <= tPC+1;
            tPC<=ID_EX_A+ID_EX_Imm;
	    VALID<=1'b1;
	    BRANCH_STATUS<=1'b1;

          end

      endcase

    end
  assign PC_OUT_EXECUTE=tPC;
endmodule 

module rv32_mem(EX_MEM_type,EX_MEM_B,clk,MEM_WB_ALUOut,EX_MEM_ALUOut,MEM_WB_IR,EX_MEM_IR);
  parameter load=3;
  parameter register=51;
  parameter immediate=19;
  parameter store=35; 
  
  input clk;
  input [6:0]EX_MEM_type;
  input [31:0] EX_MEM_B,EX_MEM_ALUOut,EX_MEM_IR;
  output reg [31:0] MEM_WB_ALUOut,MEM_WB_IR;
  
  reg [31:0] Data_Memory [0:1023]; // Data memory
  always @(posedge clk)
    begin
      case(EX_MEM_type)
        load:
          MEM_WB_ALUOut <=  Data_Memory[EX_MEM_ALUOut];
        store:
          Data_Memory[EX_MEM_ALUOut] <=  EX_MEM_B;
        register,immediate:
          MEM_WB_ALUOut <=  EX_MEM_ALUOut;
      endcase
      MEM_WB_IR<=EX_MEM_IR;
    end
endmodule

module rv32_wb(MEM_WB_ALUOut,clk,MEM_WB_IR,write_sig,address,data);

  input [31:0] MEM_WB_ALUOut;
  input [31:0] MEM_WB_IR;
  input clk;
  output reg write_sig;
  output reg [4:0]address;
  output reg [31:0]data;
  always@(posedge clk)
  begin
	  address<=MEM_WB_IR[11:7];
	  data<=MEM_WB_ALUOut;
	  write_sig<=1'b1;


  end
 
endmodule
module rv32im_top (RESET,clk);
  input RESET,clk;

  wire [31:0]IF_ID_IR,ID_EX_IR,PC,rs1,rs2,PC_OUT_DECODE,PC_OUT_FETCH,PC_OUT_EXECUTE,EX_MEM_B,MEM_WB_ALUOut,EX_MEM_ALUOut,MEM_WB_IR,EX_MEM_IR,data,rd_data1,rd_data2;
//  reg [31:0]Mem[1023:0];
  wire [6:0]ID_EX_type,EX_MEM_type;
  wire TAKEN_BRANCH,STALL,BRANCH_STATUS,VALID,read_sig,write_sig;
  wire [1:0]forward;
  wire [4:0]ID_EX_A,ID_EX_B,ID_EX_RD,address;

  Fetch_top fetch(.RESET(RESET),.clk(clk),.branch_status_exe(BRANCH_STATUS),.valid_exe(VALID),.jump_addr_exe(PC_OUT_EXECUTE),.IF_ID_IR(IF_ID_IR),.PC(PC),.TAKEN_BRANCH(TAKEN_BRANCH),.STALL(STALL));

  rv32de decode(.IF_ID_IR(IF_ID_IR),.clk(clk),.PC_IN_DECODE(PC),.PC_OUT_DECODE(PC_OUT_DECODE),.ID_EX_A(ID_EX_A),.ID_EX_B(ID_EX_B),.ID_EX_IR(ID_EX_IR),.ID_EX_type(ID_EX_type),.read_sig(read_sig),.ID_EX_RD(ID_EX_RD));

  regbank register_bank(.rd_data1(rd_data1),.rd_data2(rd_data2),.wr_data(data),.rs1(ID_EX_A),.rs2(ID_EX_B),.rd_wb(address),.rd_decode(ID_EX_RD),.write_sig(write_sig),.read_sig(read_sig),.clk(clk),.forward(forward));
 
  rv32ex execute_stage(.forward(forward),.clk(clk),.ID_EX_type(ID_EX_type),.ID_EX_IR(ID_EX_IR),.rs1(rd_data1),.rs2(rd_data2),.PC_IN_EXECUTE(PC_OUT_DECODE),.PC_OUT_EXECUTE(PC_OUT_EXECUTE),.VALID(VALID),.BRANCH_STATUS(BRANCH_STATUS),.EX_MEM_ALUOut(EX_MEM_ALUOut),.EX_MEM_ALUOut_feedback(EX_MEM_ALUOut),.EX_MEM_B(EX_MEM_B),.EX_MEM_IR(EX_MEM_IR));

  rv32_mem memory_stage(.EX_MEM_type(EX_MEM_type),.EX_MEM_B(EX_MEM_B),.clk(clk),.MEM_WB_ALUOut(MEM_WB_ALUOut),.EX_MEM_ALUOut(EX_MEM_ALUOut),.MEM_WB_IR(MEM_WB_IR),.EX_MEM_IR(EX_MEM_IR));

  rv32_wb write_back(.MEM_WB_ALUOut(MEM_WB_ALUOut),.clk(clk),.MEM_WB_IR(MEM_WB_IR),.write_sig(write_sig),.address(address),.data(data));
endmodule


