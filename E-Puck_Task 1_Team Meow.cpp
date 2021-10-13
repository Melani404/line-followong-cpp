#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 64
using namespace webots;

void calculatePID();
void motorPIDcontrol();
int readsensors(int sensnum);
void printsens();
int Read(int ni);
int calerror();
void turn(int forw,int side);
void turntjunction(int l);

DistanceSensor *S[3];
Motor *wheels[2];
Robot *robot = new Robot();


int I=0;
int previousError=0;
int motorSpeed;
float rightMotorSpeed;
float leftMotorSpeed;
int error;
float MaxSpeed=6.279; 
float BaseSpeed=0.8; 
bool allblack=false;
int prevpos=1;
float Kp = 1;  
float Ki = 0;
float Kd = 0; 
bool ydone=false;
bool dot=true;
int wrongpath=false;
int gone=0;
bool finished=false;
int tempo=0;

int main(int argc, char **argv) {

  

  int timeStep = (int)robot->getBasicTimeStep();

  char wheels_names[2][20] = {"left wheel motor", "right wheel motor"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0);
  }
  
 char dsNames[3][4] = {"gs0", "gs1", "gs2"}; 
  for (int i = 0; i < 3; i++) {
    S[i] = robot->getDistanceSensor(dsNames[i]);
    S[i]->enable(TIME_STEP);
  }

 
  while (robot->step(timeStep) != -1) {
   calerror(); 
   tempo++;
    while (robot->step(TIME_STEP) != -1 && gone==0 && (error==-100 || error==100)) { 

    wheels[0]->setVelocity(BaseSpeed);
    wheels[1]->setVelocity(BaseSpeed);
    calerror(); 
   }
   
    
   if(error==1 || error==2)prevpos=1;
   else if(error==-1 || error==-2)prevpos=2;
   
   if(error==100){
    allblack=true;
    error=0;
    turntjunction(8);
    calculatePID();
    motorPIDcontrol();
   }
   else if(error==-100){
   if(tempo>212 && dot==true){

int tt=0;
  while (robot->step(TIME_STEP) != -1  && tt!=2) {
  wheels[0]->setVelocity(-1);
  wheels[1]->setVelocity(1);
  tt++;
}

tt=0;
  while (robot->step(TIME_STEP) != -1  && tt!=46) {
  wheels[0]->setVelocity(3);
  wheels[1]->setVelocity(3);
  tt++;
}
dot=false;     
     
     
   }else
   {
    if(prevpos==1)turn(3,1);
    else if(prevpos==2)turn(3,2);
    }
   }
   else
   {
   calculatePID();
   motorPIDcontrol();
   }
    gone=1;
    
   if(finished==true){  
   wheels[0]->setVelocity(0);
  wheels[1]->setVelocity(0);
delete robot;
return 0;  
   } 
    
  
}


}

void calculatePID(){

int  P = error;
 I = I + error;
int  D = error-previousError;
     motorSpeed = (Kp*P) + (Ki*I) + (Kd*D);
     previousError = error;
}

void motorPIDcontrol(){
  
  rightMotorSpeed = BaseSpeed + motorSpeed;
  leftMotorSpeed = BaseSpeed - motorSpeed;

  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; 
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;


  wheels[0]->setVelocity(leftMotorSpeed);
  wheels[1]->setVelocity(rightMotorSpeed); 
    
}

int Read(int ni){
  int vall=readsensors(ni);
  if(vall<500){
  return 1;
  }else{
  return 0;
  }
}

int calerror(){


int val1 = Read(0);
int val2 = Read(1);
int val3 = Read(2);

    if  ((val1 == 0) && (val2 == 0)&&(val3 == 1)){error = -2;}   
 else if((val1 == 0) && (val2 == 1)&&(val3 == 1)){error = -1;}   
 else if((val1 == 0) && (val2 == 1)&&(val3 == 0)){error =  0;}   
 else if((val1 == 1) && (val2 == 1)&&(val3 == 0)){error =  1;}   
 else if((val1 == 1) && (val2 == 0)&&(val3 == 0)){error =  2;}   
 
 else if((val1 == 1) && (val2 == 0)&&(val3 == 1)){error = 6;}   
 else if((val1 == 1) && (val2 == 1)&&(val3 == 1)){error = 100;}   
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0)){error = -100;}  

 
return error;
}

int readsensors(int sensnum){
  switch (sensnum){
    case 0:
      return S[0]->getValue();
      break;
    case 1:
      return S[1]->getValue();
      break;
    case 2:
      return S[2]->getValue();
      break;
    default:
      return 0;
  }
}

void turn(int l,int side){

int tt=0;
while (robot->step(TIME_STEP) != -1  && tt!=l) {
  wheels[0]->setVelocity(3);
  wheels[1]->setVelocity(3);
  tt++;
}

if(side==1){
int tt=0;
calerror();
while (robot->step(TIME_STEP) != -1  && (error!=0)) {
  wheels[0]->setVelocity(-3);
  wheels[1]->setVelocity(3);
  tt++;
  calerror();
}
}else 

{
int tt=0;
calerror();
while (robot->step(TIME_STEP) != -1  && (error!=0)) {
  wheels[0]->setVelocity(3);
  wheels[1]->setVelocity(-3);
  tt++;
  calerror();
}
}
}

void turntjunction(int l){

int tt=0;
while (robot->step(TIME_STEP) != -1  && tt!=l) {
  wheels[0]->setVelocity(3);
  wheels[1]->setVelocity(3);
  tt++;
}

tt=0;
calerror();

if(error==100){

  tt=0;
  while (robot->step(TIME_STEP) != -1  && tt!=9) {
  wheels[0]->setVelocity(3);
  wheels[1]->setVelocity(3);
  tt++;
}

tt=0;
  while (robot->step(TIME_STEP) != -1  && tt!=22) {
  wheels[0]->setVelocity(3);
  wheels[1]->setVelocity(-3);
  tt++;
}

tt=0;
  while (robot->step(TIME_STEP) != -1  && tt!=2) {
  wheels[0]->setVelocity(0);
  wheels[1]->setVelocity(0);
  tt++;
}
finished=true;
}else{

if(tempo>700 && ydone==false){
while (robot->step(TIME_STEP) != -1  && (error!=0)) {
  wheels[0]->setVelocity(-3);
  wheels[1]->setVelocity(3);
  calerror();
}
ydone=true;
}

else{
while (robot->step(TIME_STEP) != -1) {
  wheels[0]->setVelocity(3);
  wheels[1]->setVelocity(-3);
  tt++;
  calerror();
 if(tt==15){
   wrongpath=true;
   break;
  }
  if(error==0 && tt>8){
   break;
  }
}

if(wrongpath==true){
while (robot->step(TIME_STEP) != -1  && (error!=0)) {
  wheels[0]->setVelocity(-3);
  wheels[1]->setVelocity(3);
  calerror();
}
}
wrongpath=false;
}
}
}






