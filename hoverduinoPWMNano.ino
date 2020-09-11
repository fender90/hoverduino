#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2,3);  // RX, TX
// Serial on Right Sensor cable
#define CONTROL_SERIAL_USART3
#define FEEDBACK_SERIAL_USART3
#define DEBUG_SERIAL_USART3
#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD      // [-] Start frame definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

                               // Pointer declaration for the new received data
                               
int CH1;  //PWM input channel- speed
int CH2;  //PWM input channel- steer
int hoverspeed;
int hoversteer;
int n1; 
int n2;



typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

unsigned long TimeSend = 0;



// ########################## SETUP ##########################
void setup()
{
  HoverSerial.begin(HOVER_SERIAL_BAUD);
  //HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode( 4 ,INPUT_PULLUP); //PWM input on pin 2. 1st channel is for speed
  pinMode( 5 ,INPUT_PULLUP); //PWM input on pin 3. 2nd channel is for steer
 
}


// ########################## LOOP ##########################

void loop(void)
{
  // ####### SPEED CONTROL #######
  CH1  = pulseIn(4, HIGH);
  if(( ((CH1) > (1550)) || ((1450) > (CH1)) )) //Deadband in the neutral position
  {n1=CH1;}
  else
  {n1=1500;}
  hoverspeed = (map((n1), (1100), (1900), (-1000), (1000)));
  if ((CH1) == (0)) { //Send 0 in case of contact loss
  hoverspeed = 0;
}

  // ####### STEER CONTROL #######
  CH2  = pulseIn(5
  , HIGH);
  if(( ((CH2) > (1550)) || ((1450) > (CH2)) )) //Deadband in the neutral position
  {n2=CH2;}
  else
  {n2=1500;}
  hoversteer = (map((n2), (1100), (1900), (-1000), (1000)));
  if ((CH2) == (0)) { //Send 0 in case of contact loss
  hoversteer = 0;
}
 
  
    // unsigned long timeNow = millis(); 
   //  if (TimeSend > timeNow) return;  // Send commands every 100ms
   //  TimeSend = timeNow + TIME_SEND;

  Send(hoverspeed, hoversteer);  // <---------------------------------------------- Send(Steer, Speed); -1000 bis +1000
  //Serial.println(hoverspeed);
 
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
