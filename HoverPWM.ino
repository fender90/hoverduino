#include <HoverboardAPI.h>

int ch2; // pin to connect to RC receiver
int ch3;
int  hoversteer; //steering
int  hoverspeed; //speed

int serialWrapper(unsigned char *data, int len) {
 return (int) Serial1.write(data,len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);

void setup() {

Serial1.begin(115200);

pinMode(2, INPUT); // Input pin 
pinMode(3, INPUT); // Input pin 

}

void loop() {
 
  
  ch2 = pulseIn(2, HIGH, 50000); // Check PWM impulse on ch2
  ch3 = pulseIn(3, HIGH, 50000); // Check PWM impulse on ch3
  
  
hoversteer = map(ch2,1000,2000,-2000,2000); //Map PWM to steering speed 
                                          

hoverspeed = map(ch3,1000,2000,-2000,2000); //Map PWM to acceleration speed 
                                            
if (ch2==0 || ch3==0)
{
hoversteer=0;
hoverspeed=0;
}

hoverboard.sendPWM(hoverspeed, hoversteer, PROTOCOL_SOM_NOACK);
delay(20);                                                 
 }