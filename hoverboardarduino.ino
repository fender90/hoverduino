// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2,3);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
unsigned long TimeSend = 0;



int ch2; // pin to connect to RC receiver
int ch3;
int  hoversteer; //steering
int  hoverspeed; //speed



typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");
    
  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(4, INPUT); // Input pin 
  pinMode(5, INPUT); // Input pin 

  
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (HoverSerial.available()) {
    incomingByte    = HoverSerial.read();                                 // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;   // Construct the start frame    
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
    Serial.print(incomingByte);
    return;
  #endif      
  
  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p     = (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
    *p++  = incomingByte;
    idx   = 2;  
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++  = incomingByte; 
    idx++;
  } 
  
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {    
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
  
    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
      // Print data to built-in Serial
      Serial.print("1: ");   Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  
  // Update previous states
  incomingBytePrev  = incomingByte;
}

// ########################## LOOP ##########################

void loop(void)
{
  Receive();

Serial.print("1: ");   Serial.print(ch2);
  ch2 = pulseIn(4, HIGH); // Check PWM impulse on ch2
  ch3 = pulseIn(5, HIGH); // Check PWM impulse on ch3
  
  
  hoversteer = map(ch2,1000,2000,-1000,1000); //Map PWM to steering speed 
                                          

  hoverspeed = map(ch3,1000,2000,-1000,1000); //Map PWM to acceleration speed 
                                            
  if (ch2==0 || ch3==0)
  {
  hoversteer=0;
  hoverspeed=0;
  }

  
  unsigned long timeNow = millis();
  if (TimeSend > timeNow) return;  // Send commands every 100ms
  TimeSend = timeNow + TIME_SEND;

  Send(hoversteer, hoverspeed);  // <---------------------------------------------- Send(Steer, Speed); -1000 bis +1000

  digitalWrite(LED_BUILTIN, (timeNow % 200) < 100);
}
// ########################## END ##########################
