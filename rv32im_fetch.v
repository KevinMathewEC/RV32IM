/*
 Fetch instruction
 if Branch_instruction
   if Conditional jump
    Predict Branch_target_address and Branch_status(taken/not taken)
   if unconditonal jump
    Predict Branch_target_address
 Update PC based on prediction
*/

module Program_counter_control(instruction,clk,branch_status_exe,HALTED,valid_exe,valid,TAKEN_BRANCH,rst,jump_addr_exe,br_taddr,br_inst,read_write,buffer_select,branch_predictor_select,STALL,prediction_valid_exe,LHT_index,PC,br_taddr_exe);
	/*
	Variable description
	instruction - instruction fetched from program memory
	clk - clock
	branch_status_exe - Branch status(jump taken or not) ,from execution stage
	HALTED - Halt for data hazard
	valid_exe - High if execution stage is transmitting valid data to the fetch stage
	valid -High if Branch Target Buffer has valid jump address
	TAKEN_BRANCH - High if Branch is predicted to be taken
	rst -Reset
	jump_addr_exe - Jump target address,from execution stage
	br_taddr - Branch target address,from Branch target buffer
	br_inst - Branch instruction
	read_write -Read,Write select
	buffer_select -Buffer select
	branch_predictor_select - Branch Predictor Select
	STALL - stall due to control hazard
	prediction_valid_exe - wire to BTB connected to branch_status_exe
	LHT_index - Local History Table index
	PC - Program Counter
	br_taddr_exe - wire to BTB connected to jump_addr_exe
	rptr - Read pointer for instruction_fifo
	wptr - Write pointer for instruction_fifo
	jump_cond - jump condition of current instruction
		00 : current instruction is not a jump instruction
		01 : conditional jump
		10: unconditional jump
	instruction_fifo - instruction fifo [24:1] - instruction [0] - prediction made by BP for the instruction
	*/
	parameter JAL=7'b1101111;
	parameter JALR=7'b1100111;
	parameter B_inst=7'b1100011;//conditional jump opcode
	input [31:0]instruction;
	input clk,branch_status_exe,HALTED,valid_exe,valid,TAKEN_BRANCH,rst;
        input [31:0]jump_addr_exe,br_taddr;
	output reg [23:0]br_inst;
	output reg read_write,buffer_select,branch_predictor_select,STALL,prediction_valid_exe;
        output reg [4:0]LHT_index;
	output reg [31:0]PC,br_taddr_exe;
	reg [2:0]rptr,wptr;
  	reg [1:0]jump_cond;
  	reg [24:0]instruction_fifo[7:0];
	always@(*)
	begin
		if(valid==1'b0)//cache miss
		begin
			instruction_fifo[wptr]<={instruction[31:7],TAKEN_BRANCH};
			wptr<=wptr+1'b1;
		end
	end
	always@(posedge clk or posedge rst)
	begin
      		if(rst)
        	begin
          	 rptr<=3'd0;
          	 wptr<=3'd0;
        	end
      		else
        	begin
      			if(valid_exe)//valid data from execution stage-branch target address and jump status
			begin
				if(branch_status_exe!=instruction_fifo[rptr][0])//branch mispredicted
				 STALL=1'b1;//
            			br_inst=(instruction_fifo[rptr][24:1]);
				read_write<=1'b0;//write
				buffer_select<=1'b1;//choose one of the two BTB
            			branch_predictor_select<=1'b1;
				br_taddr_exe<=jump_addr_exe;//from exe stage
				prediction_valid_exe<=branch_status_exe;//from exe stage
            			LHT_index<=br_inst[4:0];
			end
      			if(instruction[6:0]==B_inst)//conditional jump
			begin
				jump_cond<=2'b01;//conditional jump
				read_write<=1'b1;//read
				buffer_select<=1'b1;
            			branch_predictor_select<=1'b1;
				br_inst<=instruction[31:8];
            			LHT_index<=instruction[12:8];
			
			end
			else if((instruction[6:0]==JAL)||(instruction[6:0]==JALR))
			begin
				jump_cond<=2'b10;//unconditional jump
				read_write<=1'b1;//read
				buffer_select<=1'b1;
				br_inst<=instruction[31:8];
            			branch_predictor_select<=1'b0;//Branch TAKEN_BRANCH not required for unconditional jump
			
			end
			else
			begin
				jump_cond<=2'b00;//no jump
				buffer_select<=1'b0;
				branch_predictor_select<=1'b0;
			end


		
			if(HALTED)// data hazard
				PC<=PC;
			if(STALL)//control hazard
			begin
        			  if(!(instruction_fifo[rptr][0]))//branch mispredicted - not taken
					PC<=br_taddr_exe;
				  else
              				PC<=instruction_fifo[rptr][24:1];
					rptr<=rptr+1'b1;
					STALL<=1'b0;
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
							PC=br_taddr;
	  				 else
						PC=PC+1;
            			 end
				 else
					PC=PC+1;
			end
		end
         end
    end
endmodule


module Branch_target_buffer(rst,rd_wr,buffer_select,br_inst,br_taddr,br_taddr_exe,valid);
	/*
	Variable description
	rd_wr -Read/Write
	buffer_select - Buffer Select
	rst - Reset
	br_inst - 24 Msb bits of the instruction
	br_taddr_exe - Branch target address,from execution stage
	memory - Branch Target Buffer
	br_taddr - Branch Target Address
	valid - High is valid data from BTB
	i - counter
	*/
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
		for(i=0;i<1023;i=i+1)
		begin
			memory[i]=47'd0;//initialize the memory to all zeros
		end
  
        end
	    else if(buffer_select)
	     begin
		if(!rd_wr)//write
		begin
         	 memory[br_inst[9:0]]={1'b1,br_inst[23:10],br_taddr_exe};//valid,tag,branch target address
  		// $display("write %b time%d",memory[br_inst[9:0]],$time);   		       
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
		//	$display("read mem%d tag %d br_addr %d valid %d time%d",memory[br_inst[9:0]][45:32],br_inst[23:10],br_taddr,valid,$time);
 		 end
		end
	     end



endmodule




module Branch_predictor(rst,LHT_index,rd_write,TAKEN_BRANCH,prediction_valid_exe);
	/*
	Variable description
	rst - Reset
	prediction_valid_exe - Jump status,from execution stage
	LHT_index - read pointer for Local History Index table
	LHT -Local History Index Table
	LPT - Local Prediction Table
	LPT_index - Local Prediction Table read pointer
	TAKEN_BRANCH - Branch prediction
	Branch_pred - Branch prediction (2 bit)
	*/
	input rst,rd_write,prediction_valid_exe;
	input [4:0]LHT_index;
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
      	else
        begin
		if(rd_write)//read
		begin	
			LPT_index=LHT[LHT_index];
			Branch_pred=LPT[LPT_index];//2 bit TAKEN_BRANCH
		//$display("lp_in %d br_pr %d time%d",LPT_index,LPT[LPT_index],$time);

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
		//$write("lp_in %d lpt[]%d time%d",LPT_index,LPT[LPT_index],$time);
				LHT[LHT_index]={1'b1,LPT_index[3:1]};//update LHT
		//$display("updated lht %d",LHT[LHT_index]);
			end
      			else if(!prediction_valid_exe)
			begin
				LPT_index=LHT[LHT_index];
				LPT[LPT_index]=(LPT[LPT_index]-1)&&(LPT[LPT_index]!=2'b00);//update LPT
			//$write("lp_in %d lpt[]%d time%d",LPT_index,LPT[LPT_index],$time);
				LHT[LHT_index]={1'b0,LPT_index[3:1]};//update LHT
	//			$display("updated lht %d",LHT[LHT_index]);

			end
		end
        end

 end
endmodule

module Fetch_top(RESET,clk,branch_status_exe,HALTED,valid_exe,jump_addr_exe,IF_ID_IR,PC,TAKEN_BRANCH,STALL);
input RESET,clk,branch_status_exe,HALTED,valid_exe;
input [31:0]jump_addr_exe;
output reg[31:0]IF_ID_IR,PC;
output TAKEN_BRANCH,STALL;
wire read_write,buffer_select,valid,branch_predictor_select,prediction_valid_exe;
wire [23:0]br_inst;
wire [31:0]br_taddr,br_taddr_exe;
wire [3:0]LHT_index;
reg [31:0]Mem[1023:0];//instruction memory
always@(posedge clk)
begin
	IF_ID_IR<=Mem[PC];
end

Branch_target_buffer BTB_1(.rst(RESET),.rd_wr(read_write),.buffer_select(buffer_select),.br_inst(br_inst),.br_taddr(br_taddr),.br_taddr_exe(br_taddr_exe),.valid(valid));
//Branch_target_buffer BTB_2(.rst(RESET),.rd_wr(read_write),.buffer_select(buffer_select),.br_inst(br_inst),.br_taddr(br_taddr),.br_taddr_exe(br_taddr_exe),.valid(valid));

Branch_predictor BP(.rst(RESET),.LHT_index(LHT_index),.rd_write(rd_write),.TAKEN_BRANCH(TAKEN_BRANCH),.prediction_valid_exe(prediction_valid_exe));

Program_counter_control PC_control(.instruction(IF_ID_IR),.clk(clk),.branch_status_exe(branch_status_exe),.HALTED(HALTED),.valid_exe(valid_exe),.valid(valid),.TAKEN_BRANCH(TAKEN_BRANCH),.rst(RESET),.jump_addr_exe(jump_addr_exe),.br_taddr(br_taddr),.br_inst(br_inst),.read_write(read_write),.buffer_select(buffer_select),.branch_predictor_select(branch_predictor_select),.STALL(STALL),.prediction_valid_exe(prediction_valid_exe),.LHT_index(LHT_index),.PC(PC),.br_taddr_exe(br_taddr_exe));
endmodule
