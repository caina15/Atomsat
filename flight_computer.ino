#include "RTClib.h"
#include "Wire.h"
#include <SPI.h>
#include "ICM20689.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
#include <SD.h>

void mag();
void gyro();

File dataFile;
SFE_MMC5983MA myMag;

#define csmag  PA0 //mega pin 22
#define rst_ds PC2 //mega pin 35

#define PH_X  PH3 //mega pin 6
#define EN_X  PH4 //mega pin 7
#define SP_X  PL4 //mega pin 45
#define PH_Y  PH5 //mega pin 8
#define EN_Y  PH6 //mega pin 9
#define SP_Y  PL5 //mega pin 44
#define PH_Z  PB4 //mega pin 10
#define EN_Z  PB5 //mega pin 11
#define SP_Z  PL6 //mega pin 43

ICM20689 IMU(Wire,0x68);

RTC_DS3231 rtc;

int statusINM;
uint32_t currentX, currentY, currentZ = 0;  //MAG
double normalizedX, normalizedY, normalizedZ = 0; //MAG
unsigned long millisgeralpre = 0; //SD
const long intervaloSD = 500; //SD
int celsius = 0;  //MAG
int PHX, ENX, SPX = 0; //ACS X
int PHY, ENY, SPY = 0; //ACS Y
int PHZ, ENZ, SPZ = 0; //ACS Z
String receEarth, codRTC;
DateTime ajsttime;

void setup()
{
 PORTC = PORTC | (1<< PC2);
 
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(115200);
  
  while(!Serial) {}
  SPI.begin();
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
  }else{
    Serial.println("initialization done.");
  }

  // start communication with IMU
  statusINM = IMU.begin();
  if (statusINM < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("statusINM: ");
    Serial.println(statusINM);
  }
  Serial.println("temp ICM20689,X Mag axis raw value,Y Mag axis raw value,Z Mag axis raw value,X axis field (Gauss),Y axis field (Gauss),Z axis field (Gauss),ax,ay,az,gx,gy,gz,temp_C");

    if (myMag.begin(csmag) == false)
    {
      Serial.println("MMC5983MA did not respond");
    }else{
    Serial.println("MMC5983MA connected");
    }
    myMag.softReset();
}

void loop()
{
unsigned long millisgeral = millis();  
  mag();
  gyro();
  if (millisgeral - millisgeralpre >= intervaloSD) {
    millisgeralpre = millisgeral;
    dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile){
      dataFile.print(celsius);
      dataFile.print("\t");
      dataFile.print(currentX);
      dataFile.print("\t");
      dataFile.print(currentY);
      dataFile.print("\t");
      dataFile.print(currentZ);
      dataFile.print("\t");
      dataFile.print(normalizedX * 8, 5); // Print with 5 decimal places
      dataFile.print("\t");    
      dataFile.print(normalizedY * 8, 5);
      dataFile.print("\t");
      dataFile.print(normalizedZ * 8, 5);
      dataFile.print("\t");
      dataFile.print(IMU.getAccelX_mss(),6);
      dataFile.print("\t");
      dataFile.print(IMU.getAccelY_mss(),6);
      dataFile.print("\t");
      dataFile.print(IMU.getAccelZ_mss(),6);
      dataFile.print("\t");
      dataFile.print(IMU.getGyroX_rads(),6);
      dataFile.print("\t");
      dataFile.print(IMU.getGyroY_rads(),6);
      dataFile.print("\t");
      dataFile.print(IMU.getGyroZ_rads(),6);
      dataFile.print("\t");
      dataFile.print(IMU.getTemperature_C(),6);   
      dataFile.println();

      dataFile.close();
      Serial.print(F("SD write"));
    }else {
    Serial.print(F("error write"));
    }
  }  
}
void mag(){
  
  int celsius = myMag.getTemperature();
  float fahrenheit = (celsius * 9.0f / 5.0f) + 32.0f;
  Serial.print(celsius);
  Serial.print("\t"); 
  
  // This reads the X, Y and Z channels mag consecutively
  currentX = myMag.getMeasurementX();
  currentY = myMag.getMeasurementY();
  currentZ = myMag.getMeasurementZ();

  // Or, we could read all three simultaneously
  //myMag.getMeasurementXYZ(&currentX, &currentY, &currentZ);

  Serial.print(currentX);
  Serial.print("\t");
  Serial.print(currentY);
  Serial.print("\t");
  Serial.print(currentZ);
  Serial.print("\t");

  // The magnetic field values are 18-bit unsigned. The zero (mid) point is 2^17 (131072).
  // Normalize each field to +/- 1.0
  normalizedX = (double)currentX - 131072.0;
  normalizedX /= 131072.0;
  normalizedY = (double)currentY - 131072.0;
  normalizedY /= 131072.0;
  normalizedZ = (double)currentZ - 131072.0;
  normalizedZ /= 131072.0;
  
  // The magnetometer full scale is +/- 8 Gauss
  // Multiply the normalized values by 8 to convert to Gauss
  Serial.print(normalizedX * 8, 5); // Print with 5 decimal places
  Serial.print("\t");    
  Serial.print(normalizedY * 8, 5);
  Serial.print("\t");
  Serial.print(normalizedZ * 8, 5);
  Serial.print("\t");
    
}
void gyro(){
  // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getTemperature_C(),6);   
  Serial.println(); 
  }
void ACS(){
  //CONTROL ACS
  digitalWrite(SP_X, SPX);  //Set direction X
  digitalWrite(PH_X, PHX);  //enable X
  analogWrite(EN_X, ENX);   //control X
    
  digitalWrite(SP_X, SPX);  //Set direction Y
  digitalWrite(PH_X, PHX);  //enable Y
  analogWrite(EN_X, ENX);   //control Y
  
  digitalWrite(SP_X, SPX);  //Set direction Z
  digitalWrite(PH_X, PHX);  //enable Z
  analogWrite(EN_X, ENX);   //control Z
}
void RTCds(){
  
  DateTime now = rtc.now(); // get the current time
//para receber dados usar assim\/\/
/*
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();  
*/  
}
void RebDataRTC(){
  if(receEarth == codRTC){
    rtc.adjust(ajsttime); //(ANO), (MÊS), (DIA), (HORA), (MINUTOS), (SEGUNDOS)
  }
}
