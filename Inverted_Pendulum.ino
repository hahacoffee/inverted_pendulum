#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  
#define LED_PIN 13 
// global variables *********************************
int inPinE=11;
int inPin1=5;
int inPin2=6;
float detect_angle,cal_angle,angle_offset,target_angle,error_angle;
float kp,ki,kd;
int time_situation;
unsigned long Initial_time,Initial_stable_time,Start_time,Interval_time;
// DMP global variables *****************************
bool blinkState = false;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
Quaternion q;           
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3];           
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// Interrupt ******************************************
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

// functions ****************************************
void Right(float i){
  int ir,vr;
  ir = (int)(-i)+60;
  vr = ir;
  if(ir>255){vr=255;}
  if(vr>=0){
  Start_time = millis();
  digitalWrite(inPinE,HIGH);
  analogWrite(inPin1,vr);
  analogWrite(inPin2,0);
  }
  Serial.print(","); 
  Serial.print(-vr);
}
void Left(float i){
  int il,vl;
  il = (int)(i)+60;
  vl = il;
  if(il>255){vl=255;}
  if(vl>=0){
  Start_time = millis();
  digitalWrite(inPinE,HIGH);
  analogWrite(inPin1,0);
  analogWrite(inPin2,vl);  
  }
  Serial.print(","); 
  Serial.print(vl);
}
void Stop(){
  Start_time = millis();
  digitalWrite(inPinE,HIGH);
  analogWrite(inPin1,0);
  analogWrite(inPin2,0);
  Serial.print(","); 
  Serial.print("0");  
}

float Angle(float a,float ao){
  if(a<0){a=a+360;}
  a=a+ao;
  return a;
}

unsigned long Time(){
  unsigned long at,bt;
  at = millis();
  bt = at - Initial_time;
  return bt;
}

void Initial_set_time(unsigned long ast){
  unsigned long bst;
  bst = ast*1000;
  if(Time()>=bst){time_situation = 1;Start_time = millis();}
}

void Time_interval_check(unsigned long atc,unsigned long btc){
  unsigned long ctc,ttc,dtc,ttt;
  dtc = 0;
  ctc = millis();
  ttc = ctc-atc;
  if(ttc<btc){dtc = btc-ttc;delay(dtc);}
}

// PID ************************************************

class PID{
private:
  float K[3];
  float Error[3];
  float u;
public:
  PID(void){
    for(int i=0;i<3;++i){
      K[i]=0;
      Error[i]=0;
    }
    u = 0;
  }
  void PID_para(float ak,float bk,float ck){
    K[0]=ak;K[1]=bk;K[2]=ck;
  }
  void cal_PID(float tcal,float acal){
    float du,error;
    Error[0]=tcal-acal;
    error=Error[0];
    if(error<0){error=-error;}
    du=K[0]*(Error[0]-Error[1])+K[1]*Error[0]+K[2]*(Error[0]-2*Error[1]+Error[2]);
    u=u+du;
    Serial.print(","); 
    Serial.print(u); 
    Error[2]=Error[1];
    Error[1]=Error[0];
    if(error>=0 && error<0.3){Stop();}
    else if(u>0){Left(u);}
    else if(u<0){Right(u);}
    else{Stop();}
  }
};

PID angle;

// setup MPU6050 **************************************
void setup() {

// initialize global variables ************************
kp =12;
ki = 0;
kd = 2.5;
angle_offset = 2;
target_angle = 180;
Initial_stable_time = 20;
Interval_time = 10;
// ****************************************************
time_situation = 0;
angle.PID_para(kp,ki,kd);

  pinMode(inPinE,OUTPUT);
  pinMode(inPin1,OUTPUT);
  pinMode(inPin2,OUTPUT);
  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); 
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    while (Serial.available() && Serial.read()); 
    while (!Serial.available());                 
    while (Serial.available() && Serial.read()); 

    devStatus = mpu.dmpInitialize();
    
// Offset for MPU6050 *********************************

    mpu.setXGyroOffset(160);
    mpu.setYGyroOffset(17);
    mpu.setZGyroOffset(40);
    mpu.setXAccelOffset(-2860);
    mpu.setYAccelOffset(7);
    mpu.setZAccelOffset(680);
    
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    }
    pinMode(LED_PIN, OUTPUT);
    Initial_time = millis();
}

void loop() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        }  
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            detect_angle=ypr[2]*180/M_PI;
            cal_angle=Angle(detect_angle,angle_offset);  
                 
            Serial.print(cal_angle);        
        #endif

        if(time_situation == 0){
          Initial_set_time(Initial_stable_time);
          Serial.print(",");
          Serial.print("0");
          Serial.print(",");
          Serial.print("0");
        if(time_situation == 1){target_angle=cal_angle;}
        }else{    
        error_angle=target_angle-cal_angle;
        if(error_angle<0){error_angle=-error_angle;}

        if(error_angle>0.3 && error_angle<10){angle.cal_PID(target_angle,cal_angle);}
        else{Stop();Serial.print(",");Serial.print("0");}
        
        }        
        Serial.print("\n");
    }
}
