#include <SoftwareSerial.h>
#include <Servo.h>
SoftwareSerial BT(2,3);
int throttle;
int pwm;

Servo ESC1,ESC2,ESC3,ESC4;
void setup() {
  // put your setup code here, to run once:
BT.begin(9600);
Serial.begin(38400);
ESC1.attach(10,1000,2000);
ESC2.attach(9,1000,2000);
ESC3.attach(8,1000,2000);
ESC4.attach(7,1000,2000);
}

void loop() {
  // put your main code here, to run repeatedly:
if(BT.available()){
int data = BT.read();
if(data ==49){
  //Serial.println(true);
  throttle +=1 ;
  }
  
  if(data ==48){
    throttle-=1;
  //Serial.println(false);
  }
  Serial.print(throttle);  Serial.print(" ");
  if (throttle>0 and throttle<1500){throttle = throttle;}
  if (throttle<0){throttle = 0;}
  if (throttle>1500){throttle = 1500;}
  pwm = throttle;
  Serial.print(pwm);Serial.print(" ");
  pwm = map(pwm,0,1500,0,180);
    Serial.println(pwm);
  ESC1.write(pwm);
ESC2.write(pwm);
ESC3.write(pwm);
ESC4.write(pwm);
}


//Serial.println("awaiting");
}
