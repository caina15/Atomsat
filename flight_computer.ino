#include <SPI.h>

#include "ICM20689.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
#include <SD.h>

File dataFile;
SFE_MMC5983MA myMag;

#define csmag  B00000001 //PA0 (mega pin 22)
#define rst_ds B00000100 //PC2 (mega pin 35)
ICM20689 IMU(Wire,0x68);

int statusINM;
uint32_t currentX = 0;
uint32_t currentY = 0;
uint32_t currentZ = 0;
double normalizedX = 0;
double normalizedY = 0;
double normalizedZ = 0;
unsigned long millisgeralpre = 0;
const long intervaloSD = 500;
int celsius = 0;

void setup()
{
 PORTC = PORTC | (1<< PC2);
 
  Serial.begin(115200);
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
