#include<Wire.h>
#include<Servo.h>

Servo s1,s2;

byte servo_pin_1=10;  //left  blue
byte servo_pin_2=9;  //right  purple
const int MPUaddr = 0x68;
float accX, accY, accZ, gyroX;
float aAngleX=0, gAngleX=0;
float aErrorX=0, aErrorY=0, aErrorZ=0, gErrorX=0;
float pitch;
float timediff, prevtime=0, cutime=0;
int t = 0;

float pid_p ,pid_i, pid_d;
float kp=1, ki=0, kd=0;
double currenttime, previoustime, timespent;
float setpoint=0, value, error, preverr, toterr, errate, output, motor_out_1, motor_out_2;
double throttle_left = 1000;
double throttle_right = 1000; 

volatile long start, pulse_time;

void setup() {

 Serial.begin(9600);

 start = 0;
 s1.attach(servo_pin_1);  //left motor
 s2.attach(servo_pin_2);  //right right
 s1.writeMicroseconds(throttle_left);
 s2.writeMicroseconds(throttle_right);
 delay(3000);
 s1.writeMicroseconds(throttle_left + 500);
 s2.writeMicroseconds(throttle_right + 500);
 //attachInterrupt(3, receiverinterrupt, CHANGE);   //receiver pin roll channel making use of interrupt pin
 
 Wire.begin();
 Wire.beginTransmission(MPUaddr);
 Wire.write(0x6B);              //0x6B
 Wire.write(0x00);              //0x00
 Wire.endTransmission(true);

 Wire.beginTransmission(MPUaddr);
 Wire.write(0x1B);
 Wire.write(0x01); //500deg/s sesitivity
 Wire.endTransmission(true);

 Wire.beginTransmission(MPUaddr);
 Wire.write(0x1C);   
 Wire.write(0b10000000);   //+- 4g sensitivity
 Wire.endTransmission(true);

 Error();
 delay(20);

}

void loop() {
  
 //setpoint = map(pulse_time, 1000, 2000, -60, 60);  //coverting pulse_width to tiltangle range(-60 deg - 60 deg) 
 Wire.beginTransmission(MPUaddr);
 Wire.write(0x3B);
 Wire.endTransmission(false);  //restarts 
 Wire.requestFrom(MPUaddr, 6 , true);
 accX = (Wire.read()<< 8 | Wire.read())/ 4096.0;  // As per the datasheet    16384   8192  4096   2048
 accY = (Wire.read() << 8 | Wire.read())/ 4096.0;
 accZ = (Wire.read() << 8 | Wire.read())/ 4096.0;
 accX = accX - aErrorX;
 accY = accY - aErrorY;
 accZ = accZ - aErrorZ;
 aAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI);
 //Serial.println(aAngleX);

 prevtime = cutime;
 cutime = millis();
 timediff = (cutime - prevtime)/1000;
 Wire.beginTransmission(MPUaddr);
 Wire.write(0x43);
 Wire.endTransmission(false);
 Wire.requestFrom(MPUaddr, 2 , true);
 gyroX = (Wire.read() << 8 | Wire.read())/ 65.5;  // As per the datasheet
 gyroX = gyroX - gErrorX;
 gAngleX = gAngleX + (gyroX * timediff);
 //Serial.print(gAngleX);
 //Serial.print("\t");

 //complimentary filters
 pitch = 0.985 * gAngleX + 0.015 * aAngleX;
 pid(pitch, timediff, setpoint);

  Serial.print("Pitch angle : ");
  Serial.print(pitch);
  Serial.print("\t");
}

void Error()
{
  while(t < 600)
   {
     Wire.beginTransmission(MPUaddr);
     Wire.write(0x3B);
     Wire.endTransmission(false);  //restarts 
     Wire.requestFrom(MPUaddr, 6 , true);
     accX = (Wire.read()<< 8 | Wire.read())/ 4096.0;
     accY = (Wire.read()<< 8 | Wire.read())/ 4096.0;
     accZ = (Wire.read()<< 8 | Wire.read())/ 4096.0;// As per the datasheet      
     aErrorX = aErrorX + accX;
     aErrorY = aErrorY + accY;
     aErrorZ = aErrorZ + accZ;

     t++;
   }
    aErrorX = aErrorX/600;
    aErrorY = aErrorY/600;
    aErrorZ = aErrorZ/600;

     t=0;
     while(t < 800)
   {  
     Wire.beginTransmission(MPUaddr);
     Wire.write(0x43);
     Wire.endTransmission(false);
     Wire.requestFrom(MPUaddr, 2 , true);
     gyroX = (Wire.read() << 8 | Wire.read())/ 65.5;  // As per the datasheet
     gErrorX = gErrorX + gyroX;

     t++;
   }
    gErrorX = gErrorX/800;
  Serial.print("aErrorX: ");
  Serial.println(aErrorX);
  Serial.print("aErrorY: ");
  Serial.println(aErrorY);
  Serial.print("aErrorZ: ");
  Serial.println(aErrorZ);
  Serial.print("gErrorX: ");
  Serial.println(gErrorX);
    
}

// Taking pitch values as process variable
 void pid(float value,float timespent, float setpoint)
{
  error = setpoint - value;

  if(abs(error)<6)
    {
    toterr += error;
    }
   else
   {toterr = 0;}
   
  errate = (error - preverr)/timespent;

  output = kp*error + ki*toterr + kd*errate;
  motor_out_1 = map(output , -50, 50, 1900, 1100);
  motor_out_2 = map(output , -50, 50, 1100, 1900);
  if(motor_out_1>1900)
  {  motor_out_1 = 1900;
    }
  else if(motor_out_2>1900)
  {  motor_out_2 = 1900;
    }
  else if(motor_out_1<1100)
  {  motor_out_1 = 1100;
    }
  else if(motor_out_2>1900)
  {  motor_out_2 = 1100;
    }
  Serial.print("Motor 1 :  ");
  Serial.print(motor_out_1);
  Serial.print("\t");
  Serial.print("Motor 2 :  ");
  Serial.println(motor_out_2);
  actuation(motor_out_1, motor_out_2);
  preverr = error;
  previoustime = currenttime;

  Serial.print("\t");
  Serial.println(output);
}

void actuation(float out_1, float out_2)
{
  s1.writeMicroseconds(out_1);
  s2.writeMicroseconds(out_2);
}

void receiverinterrupt()
{
  if(digitalRead(3) == HIGH)
    {
      start = micros();
    }
   else
   {
     if(start != 0)
     {
      pulse_time = micros() - start;
      start = 0;
     }
   }
}