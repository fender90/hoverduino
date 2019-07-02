int ch2; // pin to connect to RC receiver
int ch3;
int  hoversteer; //steering
int  hoverspeed; //speed

void setup() {

Serial.begin(9600);

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


//Serial.write(0);
//Serial.write(0);
//Serial.write(hoversteer);
//Serial.write(hoverspeed);


Serial.write((uint8_t *) &hoversteer, sizeof(hoversteer)); //Send steering values to hoverboard motherboard
Serial.write((uint8_t *) &hoverspeed, sizeof(hoverspeed)); //Send acceleration values to hoverboard motherboard
delay(20);                                                 
 }
