#include <BMP280_DEV.h>
#include <Device.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

//BMP280
float temperature, pressure, altitude;           
BMP280_DEV bmp280; 

//MPU6050
MPU6050 mpu;


// GPS
const int RX_Pin = 4;
const int TX_Pin = 3;
const int GPS_Baud_Rate = 9600;

TinyGPSPlus gpsModule;
SoftwareSerial gpsSerialPort(RX_Pin, TX_Pin);

// Servo
Servo paco;

// DHT11
#define DHTPIN 2
#define DHTTYPE DHT11
float temp;
float humi;
DHT dht(DHTPIN, DHTTYPE);

// Variables de control
float alturaAct;
float alturaAnt;
float deAlt;
bool paraDeploy = false;

const int anguloEmergencia = 180;
const int CERRADO = 90;
const int ABIERTO = 0;

int CS;
int cable = 7;

void setup()
{
  Serial.begin(9600);

  dht.begin();
  paco.attach(6);
  paco.write(CERRADO);

  gpsSerialPort.begin(GPS_Baud_Rate);

  alturaAnt = gpsModule.altitude.meters();
 pinMode(cable, INPUT_PULLUP);
Wire.begin();
Wire.setClock(400000); // opcional, pero recomendado

mpu.initialize();
bmp280.begin(BMP280_I2C_ADDR);
bmp280.setTimeStandby(TIME_STANDBY_2000MS);
bmp280.startNormalConversion();

}

void loop()
{

CS = digitalRead(cable);

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  temp = dht.readTemperature();
  humi = dht.readHumidity();

  Serial.println("===== DHT11 =====");
  Serial.print("Temp (C): ");
  Serial.println(temp);
  Serial.print("Humedad (%): ");
  Serial.println(humi);

  if (isnan(temp) || isnan(humi)) {
    Serial.println("ERROR 1001");
    return;
  }

  float gx = ax / 16384.0;
  float gy = ay / 16384.0;
  float gz = az / 16384.0;
  float g_total = sqrt(gx * gx + gy * gy + gz * gz);

  Serial.println("===== MPU6050 =====");
  Serial.print("gX: "); Serial.println(gx, 2);
  Serial.print("gY: "); Serial.println(gy, 2);
  Serial.print("gZ: "); Serial.println(gz, 2);
  Serial.print("g_total: "); Serial.println(g_total, 2);

  while (gpsSerialPort.available() > 0)
    if (gpsModule.encode(gpsSerialPort.read()))
      displayGPSData();

  if (millis() > 5000 && gpsModule.charsProcessed() < 10)
  {
    Serial.println("ERROR 1010");
  }

  alturaAct = gpsModule.altitude.meters();
  deAlt = alturaAct - alturaAnt;

  Serial.println("===== ESTADO =====");
  Serial.print("Altura anterior (m): ");
  Serial.println(alturaAnt);
  Serial.print("Altura actual (m): ");
  Serial.println(alturaAct);
  Serial.print("Delta altura (m): ");
  Serial.println(deAlt);

  if (!paraDeploy && deAlt < -2.0) {
    paco.write(ABIERTO);
    paraDeploy = true;
    Serial.println(">>>PARACAIDAS ABIERTO <<<");
  }
 else if (CS == HIGH && !paraDeploy){
 Serial.println(">>>AVISO: PARACAIDAS ABIERTO INCORRECTAMENTE<<<");
paraDeploy = true;
 } 

else if (CS == LOW && paraDeploy == true){
paco.write(anguloEmergencia);
delay(1000);
paco.write(CERRADO);
delay(1000);
Serial.println(">>AVISO: PARACIADAS NO DESPLEGADO, REINTENTADO DESPLIEGE<<");
} 


  alturaAnt = alturaAct;

  Serial.println("==============================\n");
  delay(1000);
}

void displayGPSData()
{
  Serial.println("===== GPS =====");

  if (gpsModule.location.isValid()) {
    Serial.print("Lat: ");
    Serial.println(gpsModule.location.lat(), 6);
    Serial.print("Lon: ");
    Serial.println(gpsModule.location.lng(), 6);
    Serial.print("Alt (m): ");
    Serial.println(gpsModule.altitude.meters());
  } else {
    Serial.println("ERROR 1011");
  }

  Serial.print("Fecha: ");
  if (gpsModule.date.isValid()) {
    Serial.print(gpsModule.date.day());
    Serial.print("/");
    Serial.print(gpsModule.date.month());
    Serial.print("/");
    Serial.println(gpsModule.date.year());
  } else {
    Serial.println("ERROR 1100");
  }

  Serial.print("Hora: ");
  if (gpsModule.time.isValid()) {
    Serial.print(gpsModule.time.hour());
    Serial.print(":");
    Serial.print(gpsModule.time.minute());
    Serial.print(":");
    Serial.println(gpsModule.time.second());
  } else {
    Serial.println("ERROR 1101");
  }


  Serial.println("=====DATOS ATMOSFERICOS Y REDUNDANCIA=====");
  if (bmp280.getMeasurements(temperature, pressure, altitude))
  {   Serial.print(temperature);
      Serial.print(F("*C   "));

      Serial.print(pressure);
      Serial.print(F("hPa   "));

      Serial.print(altitude);
      Serial.println(F("m"));
 } else{
  Serial.println("ERROR 1111");
 }
}