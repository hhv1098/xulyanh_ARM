#include"Robot.h"
#include<Servo.h>
Servo Servo_1;
#define null 0
Robot_Control Robot(20000,20000,20000,20000,20000,20000);
float arr[5];
char buff[25];
int endX = 9;
int endY = 10;
int endZ =11;
float pre_theta1=0, pre_theta2 =0, pre_theta3 = 0;
unsigned long times = 0;
unsigned long Current_time, Previous_time;
void setup() {
  pinMode(endX, INPUT_PULLUP);
  pinMode(endY, INPUT_PULLUP); 
  Servo_1.attach(12);
  Servo_1.write(130);
  Serial.begin(115200);

}

void loop() {
if(Serial.available()){
  times = millis();
  Previous_time = times;
  for(int i=1; i<=5; i++){
     String pos = Serial.readStringUntil(',');
     float float_pos=pos.toFloat();
     arr[i-1] = float_pos;
     }
     if (arr[0] == 0)
     {
      Robot.Config(int(arr[1]), int(arr[2]));
     }
     else if(arr[0] == 1)
     {
      Robot.Calib_home();
     }
     else if (arr[0] == 2)
     {
       if(arr[1] == 1 )
       {
        Robot.CalibJ1();
       }
       else if(arr[1] == 2)
       {
        Robot.CalibJ2();
       }
       else if (arr[1] == 3)
       {
        Robot.CalibJ3();
       }
       else if (arr[1] == 4)
       {
       Servo_1.write(180);
       }
       else if(arr[1] ==5)
       {
       Servo_1.write(110);
       }
     }
     else if (arr[0] == 3 )
     {
      Robot.GetPosition(arr[1], arr[2], arr[3]);
     }
     else
     {
        Servo_1.write(180);
        Robot.GetPosition(arr[1], arr[2], arr[3]);
        Servo_1.write(110);
        delay(5000);
        Robot.GetPosition(0, 80, -120);
//        Robot.Classify_Object(int (arr[4]));
//        Servo_1.write(180);
//        Robot.GetPosition(0, 110, -130);
//        Servo_1.write(110);
        
//      Robot.GetPosition(arr[1], arr[2], arr[3]);
//      delay(200);
//      Robot.GetPosition(0, 90, -90);
//      Robot.GetPosition(0,33,-120);
//      Servo_1.write(180);
//      Robot.GetPosition(0, 90, -90);
//      Servo_1.write(130);
     }
     }
}
