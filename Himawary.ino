#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <SPI.h>
#include <SD.h>
#include <Battery.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

Servo servoUe;
Servo servoSita;
Battery battery;
float pi = 3.1415;
float radhi = 0;
float H =  0;//高度
float A = 0; //方位角
float t = 0; //時角
float et = -14.10;//均時差
float dlt = -14.93;//赤緯
float rad = pi/180;
float ido = 35.76; //緯度
float keido = 139.8; //経度
float nowTime = 8; //時間
int SitaAngle = 0;
int realSunAngle = 0;
int UeAngle = 0;
const int chipSelect = 10;


void initSensors()
{
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void setup() {
  // put your setup code here, to run once:
  
    Serial.begin(9600);
    servoSita.attach(5);
    servoUe.attach(6);
 
    initSensors();

   //Serial.println("hoge");
  setSunHigh(nowTime,keido,ido);
  setSunAngle(ido);
  //Serial.print("H:");
  //Serial.println(H);
  //Serial.print("A:");
  //Serial.println(A);
  //Serial.println("servoUe:");
  //Serial.println(servoUe.read());
  //Serial.println("servoSita:");
  //Serial.println(servoSita.read());

    while (!Serial) {
     ;// USBケーブルが接続されるのを待つ。この待ちループは Leonardo のみ必要。
  }
  pinMode(SS, OUTPUT);
}
    char str1[128]={0};
    
    void loop() {
      
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
    mag.getEvent(&mag_event);

    float nowAngle = 0;
    tmElements_t tm;
    battery.update();
    float voltage = battery.getVoltage();
    int percentage = battery.getPercentage();
    char* CS = battery.getChStatus();
    bool ch = battery.isCharging();
    Serial.print("battery: ");
    Serial.print(voltage);
    Serial.print("V  -> ");
    Serial.print(percentage);
    Serial.println("% ");

    if (RTC.read(tm)) {
    nowTime = tm.Hour;
     }
     else{
      if (RTC.chipPresent()) {
      //Serial.println("The DS1307 is stopped.  Please run the SetTime");
      //Serial.println("example to initialize the time and begin running.");
      //Serial.println();
    } else {
      //Serial.println("DS1307 read error!  Please check the circuitry.");
      //Serial.println();
    }
    delay(9000);
     }

    //if(Serial.available()){
      if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)){
  nowAngle = orientation.heading;
    }
   // Serial.print("nowTime:");
    //Serial.println(nowTime);
    setrealSunAngle(nowAngle,A);
    //Serial.print("nowAngle:");
    //Serial.println(nowAngle);
    setSitaAngle(realSunAngle);
    //Serial.print("realSunAngle:");
    //Serial.println(realSunAngle);
    
    /*
    for(int pos = 0; pos <= SitaAngle; pos += 1) 
  {                                  
    servoSita.write(pos);              
    delay(15);                      
  } 
  for(int ue = 0; ue <= UeAngle; ue += 1) 
  {                                  
    servoUe.write(ue);              
    delay(15);                      
  } 
*/

  if(servoUe.read()<=UeAngle){
   JairoSyoDai(servoUe,servoUe.read(),UeAngle);
  }else {
   JairoDaiSyo(servoUe,servoUe.read(),UeAngle);
  }
  
  if(servoSita.read()<=SitaAngle){
    JairoSyoDai(servoSita,servoSita.read(),SitaAngle);
  }
  else{
     JairoDaiSyo(servoSita,servoSita.read(),SitaAngle);
  }
  
  //Serial.println("kakudo");
  //Serial.println(servoUe.read());
  //Serial.println(servoSita.read());
  
   //Serial.print("Sita:");
  //Serial.println(SitaAngle);
 
  //Serial.println(servoSita.read());
  //Serial.print("Ue:");
  //Serial.println(servoUe.read());
  //Serial.print("time");
  //Serial.println(nowTime);
    
    int c = Serial.read();
    //Serial.println("end");
   // }
    delay(2000);
    
 }
  


void setSunHigh(float time,float toukei,float hokui){ //太陽の角度
  t = (time-12)/12*pi+(toukei-135)/180*pi-et*rad;
  radhi = asin(sin(hokui*rad)*sin(dlt*rad)
  +cos(hokui*rad)*cos(dlt*rad)*cos(t));
  
    
  H = radhi/pi*180;
}

void setSunAngle(float hokui){ //太陽の方位
  A = atan(cos(hokui*rad)*cos(dlt*rad)*sin(t)
        /(sin(hokui*rad)*sin(radhi)-sin(dlt*rad)));

        if(t>0 && A<0){
          A = 360-(180-(A+pi)*180/pi);
        }
        A = (A+pi)*180/pi;
             
              
}

void setrealSunAngle(float nowA,float sunA){ //現在位置から見た太陽の方位角   

  if(nowA >= sunA){
    realSunAngle = (int)(360-(nowA-sunA));
  }
  else{
    realSunAngle = (int)(sunA-nowA);
  }
}

void setSitaAngle(int an){
  
  if(0<=an&&an<90){
    SitaAngle = 90-an;
    UeAngle = H;
    }
  else if(90<=an&&an<180){
     SitaAngle = 270-an;
     UeAngle = 180-H;
     }
  else if(180<=an&&an<270){
     SitaAngle = 270-an;
     UeAngle = 180-H;
  }
  else{
     SitaAngle = 450-an;
     UeAngle = H;
  }
  
}

void JairoSyoDai(Servo ser, int nowServo,int rusltServo){
  for(int pos = nowServo; pos <= rusltServo; pos += 1) 
  {                                  
    ser.write(pos);              
    delay(15);                      
  } 
}

void JairoDaiSyo(Servo ser, int nowServo,int rusltServo){
   for(int pos = nowServo; pos >= rusltServo; pos -= 1) 
  {                                  
    ser.write(pos);              
    delay(15);                      
  } 
  
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    //Serial.write('0');
  }
  //Serial.print(number);
}


