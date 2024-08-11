#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

#define BAUDRATE 9600
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

float ReceiverValue[]={0, 0, 0, 0};
MPU6050 mpu6050(Wire);
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

uint32_t LoopTimer;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

float PIDReturn[3] ={0};
float PRateRoll = 1.3 ; float PRatePitch = PRateRoll; float PRateYaw = 4.0;
float IRateRoll = 0.04 ; float IRatePitch = IRateRoll; float IRateYaw = 0.02;
float DRateRoll = 18.0 ; float DRatePitch = DRateRoll; float DRateYaw = 0.0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
 /*
float read_receiver(int pin, int state) {
 
  float duration = 0; // Thời gian giữa các xung

  // Chờ tín hiệu thay đổi trạng thái
  while (digitalRead(pin) != state) {
  }

  // Ghi nhận thời điểm bắt đầu xung
  float pulseStart = millis();

  // Chờ cho đến khi tín hiệu chuyển sang trạng thái khác
  while (digitalRead(pin) == state) {
  }

  // Ghi nhận thời điểm kết thúc xung
  float pulseEnd = millis();

  // Tính toán thời gian giữa các xung
  duration = pulseEnd - pulseStart;

  return duration;
}
*/


void gyro_signals() {
  mpu6050.update();
  RateRoll = mpu6050.getAngleX();
  RatePitch = mpu6050.getAngleY();
  RateYaw = mpu6050.getAngleZ();

}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 1 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError) / 1;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid() {
  PrevErrorRateRoll = 0; 
  PrevErrorRatePitch = 0; 
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0; 
  PrevItermRatePitch = 0; 
  PrevItermRateYaw = 0;
}


void setup() {
  Serial.begin(BAUDRATE);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  //dong co
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  Wire.begin();
  mpu6050.begin();
  delay(250);
  mpu6050.calcGyroOffsets(true);
  ESC1.attach(3);
  ESC2.attach(5);
  ESC3.attach(6);
  ESC4.attach(9);
  LoopTimer = micros();
}

void loop() {
  gyro_signals();

  ReceiverValue[0]=pulseIn(10,HIGH);
  ReceiverValue[1]=pulseIn(11,HIGH);  
  ReceiverValue[2]=pulseIn(12,HIGH);
  ReceiverValue[3]=pulseIn(13,HIGH);

  DesiredRateRoll = 0.15 * (ReceiverValue[0] - 1380);
  DesiredRatePitch = 0.15 * (ReceiverValue[1] - 1380);
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1380);

  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;
  
  Serial.println("............");
  Serial.println(ErrorRateRoll);
  Serial.println(ErrorRatePitch);
  Serial.println(ErrorRateYaw);
  
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1]; 
  PrevItermRateRoll = PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0]; 
  PrevErrorRatePitch = PIDReturn[1]; 
  PrevItermRatePitch = PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0]; 
  PrevErrorRateYaw = PIDReturn[1]; 
  PrevItermRateYaw = PIDReturn[2];
  if (InputThrottle > 1800) InputThrottle = 1800;
  MotorInput1 = 1.024 * (InputThrottle + InputRoll - InputPitch - InputYaw);
  MotorInput2 = 1.024 * (InputThrottle + InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (InputThrottle - InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (InputThrottle - InputRoll - InputPitch + InputYaw);
  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999; 
  if (MotorInput3 > 2000) MotorInput3 = 1999; 
  if (MotorInput4 > 2000) MotorInput4 = 1999;
  int ThrottleIdle = 1180;
  if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;
  int ThrottleCutOff = 1000;
  if (ReceiverValue[2] < 1050) {
    MotorInput1 = ThrottleCutOff; 
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff; 
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }
  Serial.println(".............");
  Serial.println(InputRoll);
  Serial.println(InputPitch);
  Serial.println(InputYaw);
  
  Serial.println(".........tocdo");
 Serial.println(MotorInput1);
  Serial.println(MotorInput2);
 Serial.println(MotorInput3);
  Serial.println(MotorInput4);
  Serial.println("..........tocdo");
    Serial.println("...........cambien");
  Serial.println(RateRoll);
  Serial.println(RatePitch);
   Serial.println(RateYaw);
 Serial.println("...........cambien");
   //Serial.println("...........");
   
   // Assuming PWM output for motors are connected to pin 3, 5, 6, 9
  ESC1.write(MotorInput1);
  ESC2.write(MotorInput2);
  ESC3.write(MotorInput3);
  ESC4.write(MotorInput4);
  
  while (micros() - LoopTimer < 4000); //ESC cam 4ms 
  LoopTimer = micros();

  
   //  delay(100);
  //ReceiverValue[0]=read_receiver(10,1);
 // ReceiverValue[1]=read_receiver(11,1);  
  //ReceiverValue[2]=read_receiver(12,1);
 // ReceiverValue[3]=read_receiver(13,1);
   Serial.println("......tay cam");
  Serial.println(ReceiverValue[0]);
   Serial.println(ReceiverValue[1]);
   Serial.println(ReceiverValue[2]);
  Serial.println(ReceiverValue[3]);
  Serial.println("......tay cam");
  // delay(100);
  
}
