#include <Wire.h>
#include <Servo.h>

Servo right;
Servo left;

//MPU6050 is a 16 bit data of gyros and accelerometer.

int16_t AccX, AccY, AccZ, GyrX, GyrY, GyrZ;
float AccAngle[2]; 
float GyroAngle[2]; 
float TotalAngle[2];

///
float elapsedTime, time, timePrev;
int i;
float rtd = 180 / 3.142;       //Considering pi = 3.142
///

float PID, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

double Kp = 3.00;
double Ki = 0.002;
double Kd = 2.00;

//Intitialization:
double throttle = 1300;
float desiredAngle = 0;         //Desired angle state of rig wrt to any disturbance given

///

void setup() {
  
  Wire.begin();                  // Begin I2C communication with the controller
  Wire.beginTransmission(0x6B);  //Transmission begins to the slave address 0x6B
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);    //Ends a transmission to a slave device that was begun
  Serial.begin(250000);
  right.attach(3);                //If motors present : attach the right motor to pin(3)
  left.attach(5);                 //Left motor

///
  time = millis();                //starting time count in (milli sec)
  left.writeMicroseconds(1000);
  right.writeMicroseconds(1000);
  delay(7000);                    //Giving time for all motors to start.
  
}

void loop() {
   timePrev = time;               //The previous time is stored
   time = millis();               //Time reading starts
   
   elapsedTime = (time - timePrev)/1000;

   Wire.beginTransmission(0x68);
   Wire.write(0x3B);              //Ask for the 0x3B register- correspond to AcX
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,6,true);

   //Accelerometer:
   AccX = Wire.read()<<8|Wire.read();
   AccY = Wire.read()<<8|Wire.read();
   AccZ = Wire.read()<<8|Wire.read(); 
  

   //Raw data to Angular form using Eulers equations;// divinding by 16384 is according to data sheet;
     
    AccAngle[0] = atan((AccY / 16384.0) / sqrt(pow((AccX / 16384.0), 2) + pow((AccZ / 16384.0), 2)))*rtd;
    AccAngle[1] = atan(-1*(AccX / 16384.0) / sqrt(pow((AccY / 16384.0), 2) + pow((AccZ / 16384.0), 2)))*rtd;
   

   ///

   //Gyros:
   GyrX = Wire.read()<<8|Wire.read();
   GyrY = Wire.read()<<8|Wire.read();

   //In degrees per second according to data sheet;
   GyroAngle[0] = GyrX/131.0;
   GyroAngle[1] = GyrY/131.0;
   
   //Total Inclination:
   TotalAngle[0] = 0.96*(TotalAngle[0] + GyroAngle[0]*elapsedTime)+0.04*(AccAngle[0]);
   TotalAngle[1] = 0.96*(TotalAngle[1] + GyroAngle[1]*elapsedTime)+0.04*(AccAngle[1]);

   //To print values obtained:
   //Serial.println(TotalAngle[1]) //As we need only one axis stablity

   //PID LOOP:


   error = (TotalAngle[1] - desiredAngle);

   //Proportional Element:
   pid_p = Kp*error;
   
   //Integration Element:
   //Clamping integrator is important to aviod integral whindup!
   if(-3<error<3)
   {
   pid_i = pid_i + (Ki*error);
   }

   //Derivative Element:
   pid_d = Kd*((error - previous_error)/elapsedTime);
   //

   PID = pid_p + pid_i + pid_d;
   //To print:
   //Serial.println(PID);  
}
