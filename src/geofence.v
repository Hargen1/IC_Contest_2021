module geofence ( clk,reset,X,Y,R,valid,is_inside);
input clk;
input reset;
input [9:0] X;
input [9:0] Y;
input [10:0] R;
output reg valid;
output reg is_inside;

reg [2:0] state;
reg [2:0] n_state;

parameter IDLE  = 0;
parameter INPUT = 1;
parameter SORT  = 2;
parameter AREA  = 3;
parameter OUT   = 4;

reg [9:0] X_reg;
reg [9:0] Y_reg;
reg signed [10:0] X_vec[0:4];
reg signed [10:0] Y_vec[0:4];
reg signed [10:0] X_side[0:3];
reg signed [10:0] Y_side[0:3];
reg [10:0] R_reg[0:5];
reg [2:0] count;
reg [1:0] count_s;
reg [2:0] hold_cnt;
reg [28:0] Half_Heron;
reg [21:0] sum;
// reg hold;
reg sel;
reg signed[23:0] area;

reg [14:0] R_wire[0:1];
reg [14:0] vec_temp;
reg [14:0] side_temp;

reg [28:0] sqrt_in;
wire [14:0] root;
reg signed [10:0] X_sqrt_in;
reg signed [10:0] Y_sqrt_in;
reg [14:0] Heron[0:1];
reg [14:0] root_temp;
reg [14:0] vec_temp_temp;

integer i;

//FSM
always @(posedge clk or posedge reset) begin
	if (reset) begin
		state <= INPUT;		
	end
	else begin
		state <= n_state;
	end
end

always @(*) begin
	case(state)
		IDLE: n_state = INPUT;
		INPUT: n_state = (count == 6)? SORT : INPUT;
		SORT: n_state = (count == 5)? AREA : SORT;
		AREA: n_state = (count == 5 && hold_cnt == 0 && sel == 0)? OUT : AREA;
		OUT: n_state = IDLE;
		default: n_state = state;
	endcase
end

always @(posedge clk or posedge reset) begin
	if (reset) begin
		count <= 0;		
	end
	else begin
		case(state)
			INPUT: count <= (count == 6)? 0 : (count + 1);
			SORT: count <= (count == 5)? 0 : count + 1;
			AREA: count <= (hold_cnt == 2)? ( (sel == 0)? count + 1 : count ) : count;  // choose side vector by count_s
			OUT: count <= 0;
			default: count <= count;
		endcase
	end
end

// sel 0: calc vec ; sel 1: calc side
always @(posedge clk or posedge reset) begin
	if (reset) begin
		sel <= 0;
	end
	else begin
		case(state)
			SORT: sel <= sel + 1;
			AREA: sel <= (hold_cnt == 2)? sel + 1 : sel;
			OUT: sel <= 0;
			default: sel <= sel;
		endcase
	end
end

// sort by outer product
always @(posedge clk or posedge reset) begin
	if (reset) begin
			X_reg <= 0;
			Y_reg <= 0;	
	end
	else begin
		case(state)
			INPUT: begin
				X_reg <= (count == 0)? X : X_reg;
				Y_reg <= (count == 0)? Y : Y_reg;
			end
			default: begin
				X_reg <= X_reg;
				Y_reg <= Y_reg;
			end
		endcase
	end
end

// calc sorted points vector side by side
always @(posedge clk or posedge reset) begin
	if (reset) begin
		for(i = 0; i < 4; i = i+1) begin
			X_side[i] <= 0;
			Y_side[i] <= 0;
		end
	end
	else begin
		case(state)
			SORT: begin
				for(i = 0; i < 4; i = i+1) begin
					X_side[i] <= X_vec[i + 1] - X_vec[i];
				end
				for(i = 0; i < 4; i = i+1) begin
					Y_side[i] <= Y_vec[i + 1] - Y_vec[i];
				end
			end
			default: begin
				for(i = 0; i < 6; i = i+1) begin
					X_side[i] <= X_side[i];
					Y_side[i] <= Y_side[i];
				end
			end
		endcase
	end
end

// sort by vector outer product
always @(posedge clk or posedge reset) begin
	if (reset) begin
		for(i = 0; i < 6; i = i+1) begin
			R_reg[i] <= 0;
		end		
	end
	else begin
		case(state)
			INPUT: begin
				R_reg[count] <= R;
			end
			SORT: begin
				if ( ( X_vec[sel] * Y_vec[sel + 1] - Y_vec[sel] * X_vec[sel + 1] ) > 0 ) begin
					R_reg[sel + 1] <= R_reg[sel + 2];
					R_reg[sel + 2] <= R_reg[sel + 1];
				end
				else begin
					R_reg[sel + 1] <= R_reg[sel + 1];
					R_reg[sel + 2] <= R_reg[sel + 2];
				end
				if ( ( X_vec[sel + 2] * Y_vec[sel + 3] - Y_vec[sel + 2] * X_vec[sel + 3] ) > 0 ) begin
					R_reg[sel + 3] <= R_reg[sel + 4];
					R_reg[sel + 4] <= R_reg[sel + 3];
				end
				else begin
					R_reg[sel + 3] <= R_reg[sel + 3];
					R_reg[sel + 4] <= R_reg[sel + 4];
				end
			end
			default: begin
				for(i = 0; i < 6; i = i+1) begin
					R_reg[i] <= R_reg[i];
				end
			end
		endcase
	end
end

// vector calc & outer product
always @(posedge clk or posedge reset) begin
	if (reset) begin
		for(i = 0; i < 5; i = i+1) begin
			X_vec[i] <= 0;
			Y_vec[i] <= 0;
		end		
	end
	else begin
		case(state)
			INPUT: begin
				X_vec[count - 1] <= (count == 0)? 0 : ( X - X_reg );
				Y_vec[count - 1] <= (count == 0)? 0 : ( Y - Y_reg );
			end
			SORT: begin
				if ( ( X_vec[sel] * Y_vec[sel + 1] - Y_vec[sel] * X_vec[sel + 1] ) > 0 ) begin
					X_vec[sel] <= X_vec[sel + 1];
					Y_vec[sel] <= Y_vec[sel + 1];
					X_vec[sel + 1] <= X_vec[sel];
					Y_vec[sel + 1] <= Y_vec[sel];
				end
				else begin
					X_vec[sel] <= X_vec[sel];
					Y_vec[sel] <= Y_vec[sel];
					X_vec[sel + 1] <= X_vec[sel + 1];
					Y_vec[sel + 1] <= Y_vec[sel + 1];
				end
				if ( ( X_vec[sel + 2] * Y_vec[sel + 3] - Y_vec[sel + 2] * X_vec[sel + 3] ) > 0 ) begin
					X_vec[sel + 2] <= X_vec[sel + 3];
					Y_vec[sel + 2] <= Y_vec[sel + 3];
					X_vec[sel + 3] <= X_vec[sel + 2];
					Y_vec[sel + 3] <= Y_vec[sel + 2];
				end
				else begin
					X_vec[sel + 2] <= X_vec[sel + 2];
					Y_vec[sel + 2] <= Y_vec[sel + 2];
					X_vec[sel + 3] <= X_vec[sel + 3];
					Y_vec[sel + 3] <= Y_vec[sel + 3];
				end
			end
			default: begin
				for(i = 0; i < 5; i = i+1) begin
					X_vec[i] <= X_vec[i];
					Y_vec[i] <= Y_vec[i];
				end
			end
		endcase
	end
end

// choose side vector by count_s
always @(posedge clk or posedge reset) begin
	if (reset) begin
		count_s <= 0;
	end
	else begin
		case(state)
			AREA: count_s <= (hold_cnt == 2)? ( (sel == 1)? count_s + 1 : count_s ) : count_s;
			OUT: count_s <= 0;
			default: count_s <= count_s;
		endcase
	end
end

// hold value
always @(posedge clk or posedge reset) begin
	if (reset) begin
		hold_cnt <= 0;
	end
	else begin
		case(state)
			AREA: hold_cnt <= (hold_cnt == 2)? 0 : hold_cnt + 1;
			OUT: hold_cnt <= 0;
			default: hold_cnt <= hold_cnt;
		endcase
	end
end

// keep value until next calc
// always @(posedge clk or posedge reset) begin
// 	if (reset) begin
// 		hold <= 0;
// 	end
// 	else begin
// 		case(state)
// 			AREA: hold <= (hold_cnt == 2)? 0 : 1;
// 			OUT: hold <= 0;
// 			default: hold <= hold;
// 		endcase
// 	end
// end

// choose Heron's input & shift left 3 for float point
always @(*) begin
	case(state)
		AREA: begin
			if (sel == 0) begin
				R_wire[0] = (count == 0)? (R_reg[0]) << 3 : vec_temp_temp;
				R_wire[1] = (count == 0)? (R_reg[1]) << 3 : (side_temp);
			end
			else begin
				R_wire[0] = (count == 5)? (R_reg[5]) << 3 : (R_reg[count_s + 1]) << 3;
				R_wire[1] = (count == 5)? (R_reg[0]) << 3 : (R_reg[count_s + 2]) << 3;
			end
		end
		default: begin
			R_wire[0] = 0;
			R_wire[1] = 0;
		end
	endcase
end

// keep vector & side value
always @(posedge clk or posedge reset) begin
	if (reset) begin
		vec_temp <= 0;
		side_temp <= 0;
	end
	else begin
		case(state)
			AREA: begin
				if (hold_cnt == 0) begin
					vec_temp <= (sel == 0)? root : vec_temp;
					side_temp <= (sel == 1)? root : side_temp;
				end
				else begin
					vec_temp <= vec_temp;
					side_temp <= side_temp;
				end
			end
			default: begin
				vec_temp <= vec_temp;
				side_temp <= side_temp;
			end
		endcase
	end
end

always @(posedge clk or posedge reset) begin
	if (reset) begin
		vec_temp_temp <= 0;
	end
	else begin
		vec_temp_temp <= vec_temp;
	end
end

always @(*) begin
	root_temp = (sel == 0)? vec_temp : side_temp;
end

// Heron's input
always @(*) begin
	case(hold_cnt)
		0: begin
			Heron[0] = root + R_wire[0] + R_wire[1];
			Heron[1] = root + R_wire[0] - R_wire[1];
		end 
		1: begin
			Heron[0] = root_temp - R_wire[0] + R_wire[1];
			Heron[1] = R_wire[0] - root_temp + R_wire[1];
		end
		default: begin
			Heron[0] = 0;
			Heron[1] = 0;
		end
	endcase
end

// half Heron calc
always @(posedge clk or posedge reset) begin
	if (reset) begin
		Half_Heron <= 0;
	end
	else begin
		case(state)
			AREA: begin
				Half_Heron <= Heron[0]* Heron[1];
			end
			default: begin
				Half_Heron <= Half_Heron;
			end
		endcase
	end
end

// vector calc
always @(*) begin
	if(count == 5) begin
		X_sqrt_in = X_vec[4];
		Y_sqrt_in = Y_vec[4];
	end
	else begin
		X_sqrt_in = (sel == 0)? X_vec[count] : X_side[count_s];
		Y_sqrt_in = (sel == 0)? Y_vec[count] : Y_side[count_s];
	end
end

// choose input to sqrt
always @(*) begin
	case(state)
		AREA: begin
			case(hold_cnt)
				0: begin
					sqrt_in = (X_sqrt_in* X_sqrt_in + Y_sqrt_in* Y_sqrt_in) << 6;
				end
				1: sqrt_in =  Half_Heron;
				2: sqrt_in =  Half_Heron;
				default: sqrt_in = 0;
			endcase
		end
		default: sqrt_in = 0;
	endcase
end

DW_sqrt #(29, 0) sqrt0 (.a(sqrt_in), .root(root));

// sum of one triangle are and do rounding
always @(posedge clk or posedge reset) begin
	if (reset) begin
		sum <= 0;
	end
	else begin
		case(state)
			AREA: begin
				case(hold_cnt)
					0: sum <= 0;
					1: sum <= root;
					2: sum <= ( sum * root + 128 ) >> 8;
					default: sum <= sum;
				endcase
			end
			default: sum <= sum;
		endcase
	end
end

// area calc error of two area sums
always @(posedge clk or posedge reset) begin
	if (reset) begin
		area <= 0;
	end
	else begin
		case(state)
			AREA: begin
				if (hold_cnt == 0) begin
					if (sel == 1) begin
						area <= (count == 1)? area + sum : area - sum;
					end
					else begin
						area <= area + sum;
					end
				end
				else begin
					area <= area;
				end
			end
			OUT: area <= 0;
			default: area <= area;
		endcase
	end
end

// if area >= 0, object is inside. area < 0, object is outside.
always @(posedge clk or posedge reset) begin
	if (reset) begin
		is_inside <= 0;
		valid <= 0;
	end
	else begin
		case(state)
			OUT: begin
				valid <= 1;
				is_inside <= (area[23] == 0)? 0 : 1;
			end
			default: begin
				is_inside <= 0;
				valid <= 0;
			end
		endcase
	end
end

endmodule
