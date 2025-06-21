// RS485 Master side implementation for ESP32
#include <HardwareSerial.h>// required for esp to define uart ports 

HardwareSerial SerialRS485(2); // Instance of hardware serial for UART2

#define rs485_rx 16
#define rs485_tx 17
#define rs_mode 2 

#define baud_rate 9600// baud rate for rs485c communication line

const uint8_t start_byte = 0x64; // Start of message
const uint8_t end_byte = 0x65;   // End of message

const uint8_t master_id = 0x00;
const uint8_t slave1_id = 0x01;

// const uint8_t slave2_id = 0x02; // extra slave id

const uint8_t request1 = 0x41; // 'A'
const uint8_t request2 = 0x42; // 'B'

uint16_t masterCounter = 0; // for counting replies recieved
const unsigned long timeout_timing = 2000; // time after which send a request on line to slaves.
unsigned long lastRequestTime = 0; 

// --- Data Packet Structure & Reception Variables ---
// Packet structure assumed: START_BYTE | TARGET_ADDR | SENDER_ADDR | COUNTER | PAYLOAD_BYTE1 | PAYLOAD_BYTE2 | END_BYTE
const int msg_Size = 7; 

uint8_t receive_buffer[msg_Size];
int receive_buff_index = 0;

//for error free reception of msg 
enum ReceiveState {
  WAITING_FOR_START,
  RECEIVING_DATA    
};

ReceiveState currentReceiveState = WAITING_FOR_START; //started default in wait stage

// --- Function Prototypes ---
void sendMessage(uint8_t target_add, uint8_t sender_add, uint8_t counter1);
void processMessage(const uint8_t* msg);

// --- Main Setup Function ---
void setup() 
{
  Serial.begin(9600); 

  pinMode(rs_mode, OUTPUT);
  digitalWrite(rs_mode, LOW); // Default to receive mode, to start master in reception mode

  SerialRS485.begin(baud_rate, SERIAL_8N1, rs485_rx, rs485_tx);

  Serial.println("Master started, sending requests to nodes...");
  delay(100);

} //end of setup

// --- Main Loop Function ---
void loop() 
{
  // --- Master Request Sending Logic ---
  // request will be sent after waiting for particularr amount of time
  if (millis() - lastRequestTime > timeout_timing) {
    Serial.print("\nMaster 0x");
    Serial.print(master_id, HEX);
    Serial.print(" - RS485 Baud Rate: ");
    Serial.println(baud_rate);
    Serial.println("Request sent from master........");
    //sending msg on the line 
    sendMessage(slave1_id, master_id, (uint8_t)(masterCounter & 0xFF)); /* the truncation is performed to fit the uint16_t masterCounter into the uint8_t 
    sized COUNTER field of your defined communication protocol,
     ensuring the message packet's integrity and consistency.The & 0xFF part performs a bitwise AND operation with 0xFF (which is 0b11111111 in binary). 
     This effectively masks off any bits beyond the lower 8 bits.
    */
    lastRequestTime = millis();
  }

  // --- RS485 Data Reception Logic (State Machine) ---
  //check if messages are avialble on line
  // The Master only enables its transmitter when it intends to send data. During this time, 
  // its receiver is explicitly disabled by the MAX485. Once the transmission is complete, 
  // the Master explicitly switches its MAX485 back into receive mode, allowing it to then listen for incoming acknowledgements or other messages from the slave(s).
  //  It will not "echo" its own sent messages back into its receive buffer.

  while (SerialRS485.available()) {
    int incoming_byte_int = SerialRS485.read(); 

    if (incoming_byte_int == -1) //read() gives -1 value if there is nothing available on the serial port.
    { 
      continue; //jumps to next iteration of while loop
    }

    uint8_t byte_data = (uint8_t)incoming_byte_int; //accepting incoming data
  
  //using FSM for input data handling -------------------------------------------

    switch (currentReceiveState) //for default it's in waiting state
    {
      case WAITING_FOR_START:

        if (byte_data == start_byte) //it'll accept the data if starting bits of msg are matched. and update the state to recieving data
        {
          receive_buffer[0] = byte_data; 
          receive_buff_index = 1;       
          currentReceiveState = RECEIVING_DATA; 
        }
        break;

      case RECEIVING_DATA: //this is done after message acception
        if (receive_buff_index < msg_Size) //  msg buffer is checked for free space for storing recieved data on it
        { 
          receive_buffer[receive_buff_index] = byte_data; // data is accepted till buffer is filled up 
          receive_buff_index++;                           
          
          if (receive_buff_index == msg_Size) 
          { 
            processMessage(receive_buffer); // then filled buffer is sent for processing
            
            currentReceiveState = WAITING_FOR_START; // state is again updated to check for new msg 
            receive_buff_index = 0;  // and buffer index is again set to zero
          }
        } //end of if
        else // if buffer is fulled it's emptied to use it again
        {
          Serial.println("Master: Receive buffer overflowed or packet too long, discarding current partial packet.");
          currentReceiveState = WAITING_FOR_START; 
          receive_buff_index = 0;                 
        }
        break;
    }
  } //end of while
} // End of loop

// --- Function Definitions ---
void sendMessage(uint8_t target_add, uint8_t sender_add, uint8_t counter_val) {
  digitalWrite(rs_mode, HIGH); // Transmit mode
  delay(100); 
  //actual data transmission on the line is started for.
  SerialRS485.write(start_byte);   
  SerialRS485.write(target_add);   
  SerialRS485.write(sender_add);   
  SerialRS485.write(counter_val);  
  SerialRS485.write(0x00);         // Placeholder byte 1
  SerialRS485.write(0x00);         // Placeholder byte 2
  SerialRS485.write(end_byte);     
  
  // wait for all bytes to be passed
  SerialRS485.flush(); // Wait for transmission to complete
  delay(50);
  //switching master into reciving mode
  digitalWrite(rs_mode, LOW); // Receive mode swithcing back again on reciver side
  
  /*it's half-duplex and there's a strict turn-taking protocol (Master sends, then listens; Slave listens, then sends), 
  the other device (Master) should not be transmitting data on the line at the exact 
  moment this while loop in the Slave's sendMessage function is clearing the echo*/
  
  while(SerialRS485.available()) //this part reads serial line to clear the remaining bits to discard
  {
    SerialRS485.read(); // Read and discard any bytes
  }
} // End of sendMessage

void processMessage(const uint8_t* msg) 
{
  if (msg[0] != start_byte || msg[6] != end_byte) // checking for start and end  bits of recieved msg to determine it's correctness to defined state.
  {
    Serial.println("Master: Message Framing Error (Invalid Start/End byte or unexpected packet length).");
    return;
  }
  // each byte is separated from reciev buffer
  uint8_t target_addr = msg[1];      
  uint8_t sender_addr = msg[2];      
  uint8_t sender_counter = msg[3];   
  
  Serial.print("Master: Received from 0x"); 
  Serial.print(sender_addr, HEX); // 
  Serial.print(", Target: 0x");
  Serial.print(target_addr, HEX);
  Serial.print(", Slave Counter: ");
  Serial.print(sender_counter);   
  
  // counter is updated on master side 
  masterCounter++; 
  Serial.print(" - Master: Total Requests accepted : ");
  Serial.println(masterCounter);

} // End of processMessage
