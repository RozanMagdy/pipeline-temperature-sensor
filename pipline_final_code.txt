module pipline (input clk_50 ,input system_reset, input [7:0] data_temp ,input[1:0] parity_option , input stopbit_num ,input databits_num ,output [7:0] data_monitor , output par_flag , output stop_flag ,output [6:0] monitor0 ,output [6:0] monitor1 ,output [6:0] monitor2 ); 
wire Bus ;

 buttom RECIEVE ( .clk(clk_50) ,.reset(system_reset) , .RX(Bus) , .d_num(databits_num) ,.s_num(stopbit_num) ,.par(parity_option) , .dataout(data_monitor) ,.parity_flag(par_flag), .stopbit_flag(stop_flag) ,.seg00(monitor0),.seg11(monitor1) ,.seg22(monitor2) ) ;

UART_transmitter TRANSMIT (.clk_50Mhz(clk_50) , .reset(system_reset) ,.data(data_temp) ,.TX_bus(Bus) );





endmodule


module buttom (input clk ,input reset , input RX , input d_num , input s_num ,input [1:0] par , output  [7:0] dataout ,output  parity_flag, output stopbit_flag ,output [6:0] seg00,output [6:0] seg11 ,output [6:0] seg22 ) ;
  wire txclk,rxclk,data1,odd_par_check , data_flag ;
  //wire [7:0] dout;
  baud_rate_RX bdrt (.clk(clk),.reset(reset),.rxclk(rxclk),.txclk(txclk));
 Reciever RE (.d_num(d_num),.s_num(s_num),.par(par),.odd_par_check(odd_par_check),.even_par_check(even_par_check),.rxclk(rxclk),.reset(reset),.RX(RX),.data(data1) ,.stop_flag(stopbit_flag),.data_flag(data_flag)); 
SIPO SI ( .data(data1) ,.txclk(txclk) ,.reset(reset) ,.dout(dataout),.flag(data_flag) );
parity pr(.clk(txclk) , .reset(reset) ,.din(dataout),.parity_check(odd_par_check) ,.parity_flag(parity_flag));
bin2bcd monitor(.bin(dataout) , .seg0(seg00) , .seg1(seg11) , .seg2(seg22) );

endmodule

module Reciever (d_num,s_num,par,odd_par_check,even_par_check,rxclk,reset,RX,data,stop_flag,data_flag);
  
input d_num,s_num,rxclk,reset,RX;
input [1:0] par ;

reg [2:0] state ;
reg [2:0] count1;
reg [3:0] count2 ;
reg [3:0] count3 ;
reg [3:0] bit_counter;
reg [3:0] bit_num ;

output reg data ;
output reg odd_par_check;
output reg even_par_check;
output reg stop_flag;
output reg data_flag;



 always @ (posedge rxclk , posedge reset)
 begin if (reset) begin
   state <= 3'b000 ;
 end
 else if (reset == 1'b0) begin 
   
   case (state)
     3'b000: begin
       state <= 3'b001 ;
       count1 <= 3'b000 ;
       count2 <= 4'b0000 ;
       count3 <= 4'b0000 ;
       bit_counter <= 4'b0000 ;
       //bit_num <= 4'b0000 ;
       stop_flag <= 1'b0 ;
       data_flag <= 1'b0;
       data <= 1'b0 ;
       
       
     end
     
     3'b001: begin
     if ( RX == 1'b1) 
       begin 
         state <= 3'b001;
       end
     else if (RX == 1'b0)
       begin 
         state <= 3'b010 ;
         count1 <= 3'b000;
       end
     end
     3'b010: begin
       if (RX== 1'b0) 
         begin
           if (count1< 3'b111) 
             begin
               count1 <= count1 + 3'b001 ;
               state <= 3'b010;
             end
           else if (count1 == 3'b111) 
             begin 
               state <= 3'b011 ;
               count1 <= 3'b000;
               count2 <= 4'b0001;
               bit_counter <= 4'b0000;
               // data_flag <= 1'b1;
             end
           end
         end
        3'b011: begin
         
         //data <= RX ;
          if (count2 <= 4'b1110)
            begin
              state <= 3'b011 ;
              count2 <= count2 + 4'b0001 ;
              
            end
          else if (count2 == 4'b1111)
            begin 
              state <= 3'b100;
              data_flag <= 1'b1;
              data <= RX ;
              count2 <= 4'b0001;
              bit_counter <= bit_counter + 4'b0001;
              if (bit_counter == 4'b0000)
                begin
                  count2 <=  4'b0010 ;
                end
            end
          end
        3'b100: begin 
         // count2 <= 4'b0001;
          if (d_num == 1'b0)  // 7 bit data signal
            begin 
              bit_num <= 4'b0111;
              if ( bit_counter < bit_num )
                begin 
                  state <= 3'b011 ;
                  //count2<= 4'b0001;
              end
            else if (bit_counter == bit_num ) 
              begin 
                state <= 3'b101 ;
                count3<= 4'b0000 ;
              end
            end
            if (d_num == 1'b1) // 8 bit data signal
            begin 
              bit_num <= 4'b1000;
              if ( bit_counter < bit_num )
                begin 
                  state <= 3'b011 ;
                 // count2<= 4'b0001;
              end
            else if (bit_counter == bit_num ) 
              begin 
              // data_flag <= 1'b1;
                state <= 3'b101 ;
                count3<= 4'b0001 ;
              end
            end
          end
          3'b101: begin
           
            if (par == 2'b00)   //no parity
              begin 
                state <= 3'b110 ;
                count3 <= 4'b0000;
              end
            else if (par == 3'b01) //odd parity 
            begin 
              if (count3 < 4'b1111 )
                begin 
                  count3  <= count3 + 4'b0001 ;
                  state <= 3'b101 ;
                end
              else if ( count3 == 4'b1111) 
                begin 
                  state <= 110 ;
                  count3 <= 4'b0000;
                  odd_par_check <= RX ;
                end
              end
              if (par == 2'b10) //even parity
              begin 
                if (count3 < 4'b1111 )
                begin 
                  count3  <= count3 + 4'b0001 ;
                  state <= 3'b101 ;
                end
              else if ( count3 == 4'b1111) 
                begin 
                  state <= 110 ;
                  count3 <= 4'b0001;
                  even_par_check <= RX ;
                end
              end
            end
            3'b110: begin
              if ( RX == 0) 
                begin 
                  state <= 3'b110 ;
                end
              else if (RX ==1)
                begin
                  if (count3 <4'b1111)
                    begin 
                      count3 <= count3 + 4'b0001 ;
                      state <= 3'b110;
                      
                  end
                else if (count3 == 4'b1111 )
                  begin 
                    state <= 3'b000 ;
                  stop_flag <=1'b1; 
                  data_flag <= 1'b0;  
                    count3<= 4'b0000;
                  end
                end
              end
            endcase
          end
        end
              
              
endmodule




module SIPO ( data ,txclk ,reset ,dout ,flag );

  

output reg [7:0] dout ;

input flag ;
input data ;
//wire data ;
input txclk ;
//wire ;
input reset ;
//wire reset ;

 reg [7:0]s;
 reg [3:0] count1;
always @ (posedge txclk ,posedge reset) begin
 
 if (reset)
   begin
  s <= 8'b00000000;
  count1 <= 4'b0000;
  dout <= 8'd0 ;
 end
 else if (flag == 1'b0) begin 
    s <= 8'b00000000;
  count1 <= 4'b0000;
 
end
 else if (count1 < 4'b1000 && flag == 1'b1) begin
    count1 <=  count1 + 4'b0001 ; 
  s[7] <= data;
  s[6] <= s[7];
  s[5] <= s[6];
  s[4] <= s[5];
  s[3] <= s[4];
  s[2] <= s[3];
  s[1] <= s[2];
  s[0] <= s[1];
   
end
else if (count1 ==  4'b1000 && flag == 1'b1) begin
   count1 <=  count1 + 4'b0001 ; 
  dout <= s ;
 end
 /* else if (count1 == 4'b1001) begin
    s <=8'b00000000;
    count1 <= 4'b0000;
    end*/
end

endmodule

module parity (clk , reset ,din,parity_check ,parity_flag);
  input clk , reset , parity_check ;
  input [7:0] din ;
  output reg parity_flag ;
  reg par;
  always @ (posedge clk , posedge reset )
  begin 
    if(reset)
      begin 
        parity_flag <= 0;
      end 
    else 
      begin
        
     par= din[0] ^ din[1] ^ din[2] ^din[3] ^ din[4] ^ din[5] ^din[6] ^ din[7] ;
        if (parity_check == par)
          begin
            parity_flag <= 0;
          end
        else if (parity_check != par)
          begin 
            parity_flag <= 1 ;
          end
        end
      end
    endmodule 
        

module baud_rate_RX (clk,reset,rxclk,txclk);
 input clk,reset;
 output rxclk ,txclk ;
 reg [7:0]rx_counter = 0 ;
 reg [11:0]tx_counter =0 ;
 reg rxclk ;
 reg txclk;
 
 always@(posedge clk , posedge reset)
 begin
 tx_counter <= tx_counter + 12'b000000000001;
 rx_counter <= rx_counter + 8'b00000001;
 if (reset) 
   begin
   txclk <= 0;
   rxclk <= 0;
   tx_counter <= 0;
   rx_counter <= 0;
     end
     if(tx_counter == 12'b101000101100) //50mhz/(9600*2)
  begin
  tx_counter <= 0;
  txclk <= ~ (txclk);
  end
 if(rx_counter == 8'b10100011) //50mhz/(9600*2*16)
  begin
  rx_counter <= 0;
  rxclk <= ~ rxclk;
  end
 end
endmodule
              
              
            
  module bin2bcd(
    bin,
     seg0,seg1,seg2
    );

    
    //input ports and their sizes
    input [7:0] bin;
    //output ports and, their size
    output reg [6:0] seg0,seg1,seg2 ;
    //output [11:0] bcd;
    //Internal variables
    reg [11 : 0] bcd; 
     reg [3:0] i;   
     
     //Always block - implement the Double Dabble algorithm
     always @(bin)
        begin
            bcd = 0; //initialize bcd to zero.
            for (i = 0; i < 8; i = i+1) //run for 8 iterations
            begin
                bcd = {bcd[10:0],bin[7-i]}; //concatenation
                    
                //if a hex digit of 'bcd' is more than 4, add 3 to it.  
                if(i < 7 && bcd[3:0] > 4) 
                    bcd[3:0] = bcd[3:0] + 3;
                if(i < 7 && bcd[7:4] > 4)
                    bcd[7:4] = bcd[7:4] + 3;
                if(i < 7 && bcd[11:8] > 4)
                    bcd[11:8] = bcd[11:8] + 3;  
            end
        case(bcd[3:0]) 
        4'd0 : seg0 = 7'b1111110 ;
        4'd1 : seg0 = 7'b0110000 ;
        4'd2 : seg0 = 7'b1101101 ;
        4'd3 : seg0 = 7'b1111001 ;
        4'd4 : seg0 = 7'b0110011 ;
        4'd5 : seg0 = 7'b1011011 ;
        4'd6 : seg0 = 7'b0011111 ;
        4'd7 : seg0 = 7'b1110000 ;
        4'd8 : seg0 = 7'b1111111 ;
        4'd9 : seg0 = 7'b1111011 ;
        default : seg0 = 7'b1111110 ;  
      endcase
      
      case(bcd[7:4]) 
        4'd0 : seg1 = 7'b1111110 ;
        4'd1 : seg1 = 7'b0110000 ;
        4'd2 : seg1 = 7'b1101101 ;
        4'd3 : seg1 = 7'b1111001 ;
        4'd4 : seg1 = 7'b0110011 ;
        4'd5 : seg1 = 7'b1011011 ;
        4'd6 : seg1 = 7'b0011111 ;
        4'd7 : seg1 = 7'b1110000 ;
        4'd8 : seg1 = 7'b1111111 ;
        4'd9 : seg1 = 7'b1111011 ;
        default : seg1 = 7'b1111110 ;  
      endcase
      
      case(bcd[11:8]) 
        4'd0 : seg2 = 7'b1111110 ;
        4'd1 : seg2 = 7'b0110000 ;
        4'd2 : seg2 = 7'b1101101 ;
        4'd3 : seg2 = 7'b1111001 ;
        4'd4 : seg2 = 7'b0110011 ;
        4'd5 : seg2 = 7'b1011011 ;
        4'd6 : seg2 = 7'b0011111 ;
        4'd7 : seg2 = 7'b1110000 ;
        4'd8 : seg2 = 7'b1111111 ;
        4'd9 : seg2 = 7'b1111011 ;
        default : seg2 = 7'b1111110 ;  
      endcase
    end
                
endmodule


            
         
    


module  UART_transmitter (input clk_50Mhz , input reset ,input [7:0] data , output TX_bus );
  wire txclk ,load_flag , dout, parity_check;
  uart_tx tx (.tx_baud(txclk),.rst(reset),.din(dout),.par(parity_check),.tx_dataout(TX_bus) , .load_flag(load_flag));
  piso shift_register ( .din(data) ,.clk(txclk) ,.reset(reset) ,.load(load_flag) ,.dout(dout) );
  parity_check  par (.clk(txclk) , .reset(reset) ,.din1(data),.parity_check(parity_check) );
  baud_rate bd (.clk(clk_50Mhz),.reset(reset),.rxclk(rxclk),.txclk(txclk));
  
  
endmodule

module uart_tx  (tx_baud,rst,din,par,tx_dataout , load_flag);
  input tx_baud,din,par;
  output reg tx_dataout;
  output reg  load_flag;
  input rst ;
   integer counter1;
   parameter s0=3'b000 ;      //first case
   parameter s1=3'b001 ;       //ideal state
   parameter s2=3'b010 ;      //start bit
   parameter s3=3'b011 ;      //data
   parameter s4=3'b100 ;      // parity
        
  reg [2:0]    r_SM_Main     ;
    
   always@(posedge rst ,  counter1 )
  begin
   if (rst)  begin
    tx_dataout <= 1'b1;
    counter1=0;
          end 
         else
           r_SM_Main <= s1 ;
     begin  case (r_SM_Main)
       s1: 
      begin 
        if(counter1 == 9)
      begin    //start load flag
      load_flag <=1 ;
      end
    else if (counter1 < 10)
       begin
          tx_dataout<=1'b1;    //ideal state
          r_SM_Main <= s1 ;
       end
       
        else if(counter1 == 10)
      begin tx_dataout<=1'b0;   //start bit
      r_SM_Main <= s2 ;
      
      end
    end
    s2: begin
     
     if(counter1== 10+1)
      begin tx_dataout<=din; //data coming from piso register one bit at a time
      r_SM_Main <= s3 ;
      load_flag <=1 ;
      end
  end
    s3 :  //input data
    begin if(counter1<10+9)
      begin
        tx_dataout<=din;
        r_SM_Main <= s3 ;
      end else if (counter1==10+9)
      begin tx_dataout<=par;
        r_SM_Main <= s4 ;
        load_flag<=0;
       end
        end
      s4 :   //parity check
      begin if(counter1==10+10)
        begin tx_dataout<=1'b1;
          r_SM_Main <= s1 ;
         // load_flag<=0;
          counter1 <= 0 ;
        end
      end
     
      endcase
    end
    end 
    always @(posedge tx_baud)
    begin
    counter1=counter1+1;
  end 
  endmodule
  
  
  
  module piso ( din ,clk ,reset ,load ,dout );
    

output  reg dout ;


input [7:0] din ;
//wire [7:0] din ;
input clk ;
//wire clk ;
input reset ;
//wire reset ;
input load ;
//wire load ;
reg [7:0]temp;
reg [3:0] i ;
always @ (posedge clk,posedge  reset) begin
temp <= din;
 if (reset)
   begin
  temp <= 4'b0000;
  i<=0;
  
end
 else if (load == 1'b1)
   begin
  //temp <= din;
  //i <= i + 4'b0001 ;
 if (i < 4'b1001 )
    begin
  dout <= temp[i] ;
   i <= i + 4'b0001 ;
  end
  
  //else 
   //begin
     // dout <= 1'b0 ;
     // end

end
 else begin
 // temp <= 4'b0000;
  i<=0;
  dout <=1'b0;
 end
 end

endmodule
  
  
module parity_check (clk , reset ,din1 ,parity_check );
  input clk , reset  ;
  input [7:0] din1 ;
  output reg parity_check ;
  reg par;
  always @ (posedge clk , posedge reset )
  begin 
    if(reset)
      begin 
        // par <= HIZ;
      end 
    else 
      begin
        
     par <= din1[0] ^ din1[1] ^ din1[2] ^din1[3] ^ din1[4] ^ din1[5] ^din1[6] ^ din1[7] ;
       parity_check <= par ;
        end
      end
    endmodule 
        

module baud_rate (clk,reset,rxclk,txclk);
 input clk,reset;
 output rxclk ,txclk ;
 reg [7:0]rx_counter = 0 ;
 reg [11:0]tx_counter =0 ;
 reg rxclk ;
 reg txclk;
 
 always@(posedge clk , posedge reset)
 begin
 tx_counter <= tx_counter + 12'b000000000001;
 rx_counter <= rx_counter + 8'b00000001;
 if (reset) 
   begin
   txclk <= 0;
   rxclk <= 0;
   tx_counter <= 0;
   rx_counter <= 0;
     end
     if(tx_counter == 12'b101000101100) //50mhz/(9600*2)
  begin
  tx_counter <= 0;
  txclk <= ~ (txclk);
  end
 if(rx_counter == 8'b10100011) //50mhz/(9600*2*16)
  begin
  rx_counter <= 0;
  rxclk <= ~ rxclk;
  end
 end
endmodule




