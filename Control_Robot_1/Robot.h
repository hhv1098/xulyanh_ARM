#include<MultiStepper.h>
#include<AccelStepper.h>
AccelStepper stepperX(1,2,5);
AccelStepper stepperY(1,3,6);
AccelStepper stepperZ(1,4,7);
class Robot_Control{
  private:
  const float pi = 3.14159;
  float Gear_ratio1 = 4.5;
  float Gear_ratio2 = 4.5;
  float Gear_ratio3 = 4.5; 
  //float ref_theta1 = -(97.5+15);
  float ref_theta1 = -(81.5);
  float ref_theta2 = (137);
  float ref_theta3 = -(114.2);
  float theta1 = 0, theta2 = 0, theta3 =0;
  int en = 8;
  int dirX = 5;
  int stepX = 2;
  int endX = 9;
  int dirY = 6;
  int stepY = 3;
  int endY = 10;
  int dirZ = 7;
  int stepZ = 4;
  int endZ =11;
  public: 
      Robot_Control(int vx, int ax, int vy, int ay, int vz, int az)
  {
        pinMode(en, OUTPUT);
        digitalWrite(en, LOW);
        pinMode(stepX, OUTPUT);
        pinMode(dirX, OUTPUT);
        pinMode(stepY, OUTPUT);
        pinMode(dirY, OUTPUT);
        pinMode(stepZ, OUTPUT);
        pinMode(dirZ, OUTPUT);
        stepperX.setMaxSpeed(vx);
        stepperX.setSpeed(vx);
        stepperX.setAcceleration(ax);

        stepperY.setMaxSpeed(vy);
        stepperY.setSpeed(vy);
        stepperY.setAcceleration(ay);
    
        stepperZ.setMaxSpeed(vz);
        stepperZ.setSpeed(vz);
        stepperZ.setAcceleration(az);

        pinMode(endX, INPUT_PULLUP);
        pinMode(endY, INPUT_PULLUP);
        pinMode(endZ, INPUT_PULLUP);
        stepperX.setCurrentPosition(0);
        stepperY.setCurrentPosition(0);
        stepperZ.setCurrentPosition(0);
  }
  void Config( int v,int a){
        stepperX.setMaxSpeed(v);
        stepperX.setSpeed(v);
        stepperX.setAcceleration(a);

        stepperY.setMaxSpeed(v);
        stepperY.setSpeed(v);
        stepperY.setAcceleration(a);
    
        stepperZ.setMaxSpeed(v);
        stepperZ.setSpeed(v);
        stepperZ.setAcceleration(a);
  }
  void CalibJ1()
  {
      digitalWrite(dirX,HIGH);
      while(digitalRead(endX))
      {
        digitalWrite(stepX, HIGH);
        delayMicroseconds(1000);  
        digitalWrite(stepX, LOW);
        delayMicroseconds(1000);  
      }
      stepperX.setCurrentPosition(0); 
  }
  void CalibJ2()
  {
    digitalWrite(dirY, HIGH);
    while(digitalRead(endY))
    {
      digitalWrite(stepY, HIGH);
      delayMicroseconds(1000);  
      digitalWrite(stepY, LOW);
      delayMicroseconds(1000);  
    }
    stepperY.setCurrentPosition(0);  
  }
  void CalibJ3()
  {
    digitalWrite(dirZ,LOW);
    while(digitalRead(endZ))
    {
      digitalWrite(stepZ, HIGH);
      delayMicroseconds(1000);  
      digitalWrite(stepZ, LOW);
      delayMicroseconds(1000);  
    }   
    stepperZ.setCurrentPosition(0);
  }
  void Gripper_On(){
  }
  void Gripper_Off(){
  }
    
  void GetPosition(float theta1, float theta2, float theta3){
          int pulse_1 = Gear_ratio1*(theta1-ref_theta1)*200*16/360;
          int pulse_2 = Gear_ratio2*(theta2-ref_theta2)*200*16/360;
          int pulse_3 = Gear_ratio3*(theta3-ref_theta3)*200*16/360; 
          stepperX.moveTo(-pulse_1);
          stepperY.moveTo(pulse_2);
          stepperZ.moveTo(-(pulse_3+pulse_2));
          while((stepperX.distanceToGo()!=0)||(stepperY.distanceToGo()!=0)||(stepperZ.distanceToGo()!=0))
          {
             stepperX.run();
             stepperY.run();
             stepperZ.run();
           }
  }
  void Calib_home(){
    CalibJ1();
    CalibJ3();
    CalibJ2();
    GetPosition(0,110,-130);      
  }
  void Inverse(float px, float py, float pz, float *theta1, float *theta2, float *theta3){
    float d1 = 9.6, a2 = 13.5, a3 = 14.7, a4 = 5.2, a5 = 6;
    *theta1 = atan2(py,px);
    px = px - a4*cos(*theta1);
    py = py - a4*sin(*theta1);
    pz = pz + a5;
    float r = sqrt(pow(px,2) + pow(py,2) + pow(pz-d1,2));
    *theta3 = -acos((pow(r,2) - pow(a2,2)- pow(a3,2))/(2*a2*a3));
    *theta2 = asin((pz-d1)/r) +  atan(a3*sin(abs(*theta3))/( a2 + a3*cos(abs(*theta3)))); 
    *theta1 = round(*theta1*180*100/pi)/100;
    *theta2 = round(*theta2*180*100/pi)/100;
    *theta3 = round(*theta3*180*100/pi)/100;
  }
  void Path_Planning(float point_A[3], float point_B[3],float point_C[3], int type){
    if(type)
    {
      float s = sqrt(pow(point_B[0] - point_A[0],2)+pow(point_B[1] - point_A[1],2)+pow(point_B[2] - point_A[2],2));
      float kx = (point_B[0]-point_A[0])/s;
      float ky = (point_B[1]-point_A[1])/s;
      float kz = (point_B[2]-point_A[2])/s;
      for (int i=0; i<=30; i++)
      {
        float x = point_A[0] + kx*(s/30)*i;
        float y = point_A[1] + ky*(s/30)*i;
        float z = point_A[2] + kz*(s/30)*i; 
        Inverse(x, y, z, &theta1, &theta2, &theta3); 
        Serial.println(theta1);
        Serial.println(theta2);
        Serial.println(theta3);
        GetPosition( theta1, theta2, theta3);    
      }  
    }
    else
    {
      float a1 = pow(point_B[0]-point_C[0],2) + pow(point_B[1]-point_C[1],2);
      float b1 = pow(point_A[0]-point_C[0],2) + pow(point_A[1]-point_C[1],2);
      float c1 = pow(point_A[0]-point_B[0],2) + pow(point_A[1]-point_B[1],2);
      float x_TS = a1*(b1+c1-a1)*point_A[0] + b1*(a1+c1-b1)*point_B[0] + c1*(a1+b1-c1)*point_C[0];
      float y_TS = a1*(b1+c1-a1)*point_A[1] + b1*(a1+c1-b1)*point_B[1] + c1*(a1+b1-c1)*point_C[1]; 
      float MS = a1*(b1+c1-a1) + b1*(a1+c1-b1) + c1*(a1+b1-c1);
      float x_O = x_TS/MS;
      float y_O = y_TS/MS;
      float R = sqrt(pow(x_O-point_A[0],2) + pow(y_O-point_A[1],2));
      float phi1 = atan2(point_A[1]-y_O, point_A[0]-x_O);
      float phi2 = atan2(point_B[1]-y_O, point_B[0]-x_O);
      float phi3 = atan2(point_C[1]-y_O, point_C[0]-x_O);
      float phi_max = max(phi1,phi2);
      phi_max = max(phi_max,phi3);
      float phi_min = min(phi1,phi2);
      phi_min = min(phi_min,phi3);
      for (int i=0;i<=50;i++)
      {
        float x = R*cos(phi_min+(phi_max - phi_min)*i/50) + x_O;
        float y = R*sin(phi_min+(phi_max - phi_min)*i/50) + y_O;
        float z = 5.2;
        Inverse(x, y, z, &theta1, &theta2, &theta3);
        Serial.println(theta1);
        Serial.println(theta2);
        Serial.println(theta3);
        Serial.println("/////");
        GetPosition(theta1, theta2, theta3);
      }
      
    }
  }
  void Classify_Object(int flag){
  switch (flag)
  {
    case 0:
      GetPosition(37,23,-105);
      break;
    case 1:
      GetPosition(29.5,23,-112);
      break;
    case 2:
      GetPosition(20.5,23,-117);
      break;
    default:
      break;    
  }
//      GetPosition(0, 110, -130);
  }
};
