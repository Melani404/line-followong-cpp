#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 32
using namespace webots;


//line following stuff
int calerror();
void motorPIDcontrol();
void calculatePID();
void stop();
void tjunction();
double readsensors(int sensnum);
int Read(int ni);
void turn(int side);
int corrections(float distancetopillar);
void pillarcalculation(int pillarside);

//int irVal[7];
int I=0;
float val;
float MaxSpeed=15; //41
float BaseSpeed=10; //32   34 medium   25 safest  40 speed
float lastPosition=0;
int error =-100;
int previousError=0;
int motorSpeed;
int gone=0;
float rightMotorSpeed;
float leftMotorSpeed;
bool finished=false;
double lefts;
double rights;
int leftcount=0;
int rightcount=0;
int nopillar;

bool shortleft=false;
float pillardistance;

float Kp = 1;   // Harima Wedanai Appa ;-)
float Ki = 0;
float Kd = 0; 


//------------WALL STUFF--------------//

void wallfollowing();
void wallprintsens();
void wallmotorPIDcontrol();
void wallcalculatePID();
double readsensors(int sensnum);

int wallI=0;
int wallMaxSpeed=12; //41
int wallBaseSpeed=6; //32   35 medium   25 safest  40 speed
float wallerror =0;
int wallpreviousError =0;
int wallmotorSpeed;
float wallrightMotorSpeed;
float wallleftMotorSpeed;

float wallKp = 1;   // Harima Wedanai Appa ;-)
float wallKi = 0; 
float wallKd = 0.001; 

bool walldone=false;

//**********varibales and sttuff***************


Robot *robot = new Robot();
DistanceSensor *S[9];
DistanceSensor *TS[9];
Motor *wheels[2];



//********************************************



int main(int argc, char **argv) {
  
/*********************INIZIALING**************************/ 
 

  
//MOTORS
  char wheels_names[2][7] = {"leftm", "rightm"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY); 
    wheels[i]->setVelocity(0); 
  } 
 

  
  
  
  
  
//********************LOOOOOOOP**************************/  
  
  
  while (robot->step(TIME_STEP) != -1) {
  
    wheels[0]->setVelocity(BaseSpeed);
    wheels[1]->setVelocity(BaseSpeed);
  //STAGE 1
  
 /*  for (int i = 0; i < 7; i++) {
    std::cout << TS[i]->getValue() << "      "; 
  }*/
  
/*   calerror();
   
  while (robot->step(TIME_STEP) != -1 && gone==0 && error==-100) { //black get away

    wheels[0]->setVelocity(BaseSpeed);
    wheels[1]->setVelocity(BaseSpeed);
    calerror(); 
   }
   
 
   if(error==10){
   pillarcalculation(1);//left
   turn(1);
   }
   else if(error==-10){
   pillarcalculation(2);//right
   turn(2);
   }
   else if(error==-100)tjunction();
   else if(error==100)wallfollowing();
   
   gone=1;
   
   calculatePID();
   motorPIDcontrol();*/
  
  
   }
   
/***********************************************************************************/   
   
}







int calerror(){
/* 
 
 Sensor Array       Error Value
   0000001             -6
   0000011             -5
   0000111             -4
   0001111             -3
   0001110             -2
   0011110             -1
   0011100              0
   0111100              1
   0111000              2
   1111000              3
   1110000              4
   1100000              5
   1000000              6

*/
int val1 = Read(0);
int val2 = Read(1);
int val3 = Read(2);
int val4 = Read(3);
int val5 = Read(4);
int val6 = Read(5);
int val7 = Read(6); 
int val8 = Read(7);
int val9 = Read(8); 
std::cout<< "Error - " << error << std::endl;

    if  ((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 0)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 1)){error = 8;}   //000000001
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 0)&& (val6 == 0)&& (val7 == 0)&& (val8 == 1)&& (val9 == 1)){error = 7;}   //000000011
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 0)&& (val6 == 0)&& (val7 == 1)&& (val8 == 1)&& (val9 == 1)){error = 6;}   //000000111
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 0)&& (val6 == 1)&& (val7 == 1)&& (val8 == 1)&& (val9 == 1)){error = 5;}   //000001111
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 0)&& (val6 == 1)&& (val7 == 1)&& (val8 == 1)&& (val9 == 0)){error = 4;}   //000001110
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 1)&& (val6 == 1)&& (val7 == 1)&& (val8 == 1)&& (val9 == 0)){error = 3;}   //000011110
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 1)&& (val6 == 1)&& (val7 == 1)&& (val8 == 0)&& (val9 == 0)){error = 2;}   //000011100
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 1)&&(val5 == 1)&& (val6 == 1)&& (val7 == 1)&& (val8 == 0)&& (val9 == 0)){error = 1;}   //000111100
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 1)&&(val5 == 1)&& (val6 == 1)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = 0;}   //000111000
 else if((val1 == 0) && (val2 == 0)&&(val3 == 1) && (val4 == 1)&&(val5 == 1)&& (val6 == 1)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = -1;}  //001111000
 else if((val1 == 0) && (val2 == 0)&&(val3 == 1) && (val4 == 1)&&(val5 == 1)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = -2;}  //001110000
 else if((val1 == 0) && (val2 == 1)&&(val3 == 1) && (val4 == 1)&&(val5 == 1)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = -3;}  //011110000
 else if((val1 == 0) && (val2 == 1)&&(val3 == 1) && (val4 == 1)&&(val5 == 0)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = -4;}  //011100000
 else if((val1 == 1) && (val2 == 1)&&(val3 == 1) && (val4 == 1)&&(val5 == 0)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = -5;}  //111100000
 else if((val1 == 1) && (val2 == 1)&&(val3 == 1) && (val4 == 0)&&(val5 == 0)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = -6;}  //111000000
 else if((val1 == 1) && (val2 == 1)&&(val3 == 0) && (val4 == 0)&&(val5 == 0)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = -7;}  //110000000
 else if((val1 == 1) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 0)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = -8;}  //100000000
 
 /*************************************************************************************************************************************************************************/    
 
 /*****************************************************************************************************************************************************************************************************************************************************************************************************************/
 else if((val1 == 1) && (val2 == 1)&&(val3 == 1) && (val4 == 1)&& (val5 == 1)&& (val6 == 1)&&(val9 == 0)){error = -10;}    //111111##0
 else if((val1 == 0) && (val2 == 0)&&(val5 == 1) && (val6 == 1)&& (val7 == 1)&& (val8 == 1)&&(val9 == 1)){error = 10;}     //0##111111
 else if((val1 == 1) && (val2 == 1)&&(val3 == 1) && (val4 == 1)&&(val5 == 1)&& (val6 == 1)&& (val7 == 1)&& (val8 == 1)&& (val9 == 1)){error = -100;}   //111111111
 else if((val1 == 0) && (val2 == 0)&&(val3 == 0) && (val4 == 0)&&(val5 == 0)&& (val6 == 0)&& (val7 == 0)&& (val8 == 0)&& (val9 == 0)){error = 100;}    //000000000

return error;
}



int Read(int ni){
  int vall=readsensors(ni);
  std::cout << vall << "  ";
  if(vall<60){
  return 1;
  }else{
  return 0;
  }
}


double readsensors(int sensnum){
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
    case 3:
      return S[3]->getValue();
      break;
    case 4:
      return S[4]->getValue();
      break;
    case 5:
      return S[5]->getValue();
      break;
    case 6:
      return S[6]->getValue();
      break;
    case 7:
      return S[7]->getValue();
      break;
    case 8:
      return S[8]->getValue();
      break;
    default:
      return 0;
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

  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;


  wheels[0]->setVelocity(leftMotorSpeed);
  wheels[1]->setVelocity(rightMotorSpeed); 
    
}


void turn(int side){

int tt=0;
while (robot->step(TIME_STEP) != -1  && tt!=4) {
  wheels[0]->setVelocity(10);
  wheels[1]->setVelocity(10);
  //checkpillar();
  tt++;
}

calerror();

if(error==-100){
 tjunction();
}

else if(side==2){
tt=0;
while (robot->step(TIME_STEP) != -1  && tt!=14) {
  wheels[0]->setVelocity(10);
  wheels[1]->setVelocity(-10);
  //checkpillar();
  tt++;
}
}
else if(side==1)
{
tt=0;
while (robot->step(TIME_STEP) != -1  && tt!=14 ) {
  wheels[1]->setVelocity(10);
  wheels[0]->setVelocity(-10);
  //checkpillar();
  tt++;
}
}
calerror();
/*tt=0;
while (robot->step(TIME_STEP) != -1  && tt!=1 ) {
  wheels[1]->setVelocity(BaseSpeed);
  wheels[0]->setVelocity(BaseSpeed);
  //checkpillar();
  tt++;
}*/

}

void tjunction(){

if(stage==1){//             STAGE 1
   calerror();
  
   lefts=TS[2]->getValue();
   rights=TS[3]->getValue();
   
   if(lefts<rights){
      shortleft=true;
      std::cout << "Left side pillar is the closest";
   }else{
      std::cout << "Right side pillar is the closest";
   }

   while (robot->step(TIME_STEP) != -1 && error==-100 && error!=10 && error!=-10) {

    wheels[0]->setVelocity(BaseSpeed);
    wheels[1]->setVelocity(BaseSpeed);
    calerror(); 
   }
   
  int tt=0;
  while (robot->step(TIME_STEP) != -1  && tt!=10) {
  wheels[0]->setVelocity(BaseSpeed);
  wheels[1]->setVelocity(BaseSpeed);
  //checkpillar();
  tt++;
}
   calerror();
   std::cout << "Stepooooooooooooooo";
   stage=2;
  }
  else if(stage==2){//      STAGE 2
  
      if(shortleft==true)turn(1);
      else if(shortleft==false)turn(2);
  }
}





//*****************************************************************************

double wallreadsensors(int wallsensnum){
  switch (wallsensnum){
    case 0:
      return TS[0]->getValue();
      break;
    case 1:
      return TS[1]->getValue();
      break;
    case 2:
      return TS[2]->getValue();
      break;
    case 3:
      return TS[3]->getValue();
      break;
    case 4:
      return TS[4]->getValue();
      break;
    case 5:
      return TS[5]->getValue();
      break;
    case 6:
      return TS[6]->getValue();
      break;
    case 7:
      return TS[7]->getValue();
      break;
    case 8:
      return TS[8]->getValue();
      break;
    default:
      return 0;
  }
}

void wallcalculatePID(){
  
int  wallP = wallerror;
 wallI = wallI + wallerror;
int  wallD = wallerror- wallpreviousError;
     wallmotorSpeed = (wallKp*wallP) + (wallKi*wallI) + (wallKd*wallD);
     wallpreviousError = wallerror;
}

void wallmotorPIDcontrol(){
  
  wallrightMotorSpeed = wallBaseSpeed + wallmotorSpeed;
  wallleftMotorSpeed = wallBaseSpeed - wallmotorSpeed;
  
  if (wallrightMotorSpeed > wallMaxSpeed ) wallrightMotorSpeed = wallMaxSpeed; // prevent the motor from going beyond max speed
  if (wallleftMotorSpeed > wallMaxSpeed ) wallleftMotorSpeed = wallMaxSpeed; // prevent the motor from going beyond max speed
  if (wallrightMotorSpeed < 0)wallrightMotorSpeed = 0;    
  if (wallleftMotorSpeed < 0)wallleftMotorSpeed = 0;


  wheels[1]->setVelocity(wallleftMotorSpeed);
  wheels[0]->setVelocity(wallrightMotorSpeed); 
    
}


void wallprintsens(){
for (int i = 0; i < 7; i++) {
    std::cout << wallreadsensors(i) << "    ";
  } 
std::cout << std::endl;
}

void wallfollowing(){

 int inwall=true;
 
 if(inwall==true){
 
  calerror();
  while (robot->step(TIME_STEP) != -1 && error==100 && (wallreadsensors(2)<30 || wallreadsensors(3)<30)) {
    std::cout <<  " INN ";
    wallerror=(wallreadsensors(1)+5)-wallreadsensors(2); 
    std::cout << wallreadsensors(1) << "    "<< wallreadsensors(2) << "    ";
    std::cout << wallerror << "    ";
    wallcalculatePID();
    wallmotorPIDcontrol();
    calerror(); 
   }

 }
  error=0;//dotted
}



/***************************************************/

void pillarcalculation(int pillarside){


if(pillarside==1){
  pillardistance=TS[3]->getValue();
  nopillar=corrections(pillardistance);
  if(nopillar>5){
  std::cout << "Pillar at Right side - " << corrections(pillardistance)<< "cm";
  }
}else if(pillarside==2){
  pillardistance=TS[2]->getValue();
  nopillar=corrections(pillardistance);
  if(nopillar>5){
  std::cout << "Pillar at Left side - " << corrections(pillardistance)<< "cm";
}
}
}


int corrections(float distancetopillar){
if(distancetopillar>11 && distancetopillar<15.5)return 13;
else if(distancetopillar>=15.5 && distancetopillar<20.5)return 18;
else if(distancetopillar>=20.5 && distancetopillar<25.5)return 23;
else return 0;
}









