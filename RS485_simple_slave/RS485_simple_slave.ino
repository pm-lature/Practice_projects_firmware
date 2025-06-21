// RS485 Slave side implementation for ESP32
#include <HardwareSerial.h> //required for esp to define uart ports 

HardwareSerial SerialRS485(2); // Instance of hardware serial for UART2

#define rs485_rx 16
#define rs485_tx 17
#define rs_mode 2 

#define baud_rate 9600// baud rate for rs485c communication line

const uint8_t start_byte = 0x64; // Start of message
const uint8_t end_byte = 0x65;   // End of message

const uint8_t master_id = 0x00;
const uint8_t slave1_id = 0x01; // This device's ID
const uint8_t slave2_id = 0x02;

const uint8_t request1 = 0x41; // 'A'
const uint8_t request2 = 0x42; // 'B'

uint16_t slaveCounter = 0; // for counting replies recieved
unsigned long lastRequestTime = 0; 
const unsigned long timeout_timing = 2000; // time after which send a request on line to slaves.

// --- Data Packet Structure & Reception Variables --- same as master devices
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
void setup() {
  Serial.begin(9600); 

  pinMode(rs_mode, OUTPUT);
  digitalWrite(rs_mode, LOW); // Default to receive mode
  
  SerialRS485.begin(baud_rate, SERIAL_8N1, rs485_rx, rs485_tx);

  Serial.println("Slave 1 started, waiting for master requests...");
  delay(100);
}//end of setup

// --- Main Loop Function ---
void loop()
{
  
  // --- RS485 Data Reception Logic (State Machine) ---
  //check if messages are avialble on line
  // The Master only enables its transmitter when it intends to send data. During this time, 
  // its receiver is explicitly disabled by the MAX485. Once the transmission is complete, 
  // the Master explicitly switches its MAX485 back into receive mode, allowing it to then listen for incoming acknowledgements or other messages from the slave(s).
  //  It will not "echo" its own sent messages back into its receive buffer.

  while (SerialRS485.available()) 
  {
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

      case RECEIVING_DATA:
        if (receive_buff_index < msg_Size) //  msg buffer is checked for free space for storing recieved data on it
        {
          receive_buffer[receive_buff_index] = byte_data; // data is accepted till buffer is filled up 
          receive_buff_index++;                           
          
          if (receive_buff_index == msg_Size) 
          { 
            processMessage(receive_buffer); // then filled buffer is sent for processing
            
            currentReceiveState = WAITING_FOR_START; // state is again updated to check for new msg 
            receive_buff_index = 0; // and buffer index is again set to zero
          }
        } //end of if
        else // if buffer is fulled it's emptied to use it again
        {
          Serial.println("Slave1: Receive buffer overflowed or packet too long, discarding current partial packet.");
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
  digitalWrite(rs_mode, LOW); // Receive mode
  
  /*it's half-duplex and there's a strict turn-taking protocol (Master sends, then listens; Slave listens, then sends), 
  the other device (Master) should not be transmitting data on the line at the exact 
  moment this while loop in the Slave's sendMessage function is clearing the echo*/
  
  while(SerialRS485.available()) // NEW: Clear RX buffer of any echoed bytes that might have slipped through
  {
    SerialRS485.read(); // Read and discard any bytes
  }
} // End of sendMessage

void processMessage(const uint8_t* msg) {
  if (msg[0] != start_byte || msg[6] != end_byte) // checking for start and end  bits of recieved msg to determine it's correctness to defined state.
  {
    Serial.println("Slave1: Message Framing Error (Invalid Start/End byte or unexpected packet length).");
    return;
  }
  // each byte is separated from reciev buffer
  uint8_t target_addr = msg[1];      
  uint8_t sender_addr = msg[2];      
  uint8_t master_counter_received = msg[3]; 

  Serial.print("Slave1: Received from 0x"); 
  Serial.print(sender_addr, HEX); 
  Serial.print(", Master Counter: ");
  Serial.print(master_counter_received);   
  
  // counter is updated on master side 
  slaveCounter++; 
  Serial.print(" - Slave1: accepted acknoldegement counter: "); 
  Serial.println(slaveCounter);
  // send acknoledgment again onto the line
  sendMessage(master_id, slave1_id, (uint8_t)(slaveCounter & 0xFF)); 
} // End of processMessage
