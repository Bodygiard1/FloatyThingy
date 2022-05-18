#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
int sensorPin = A0;
float volt;
float ntu;
#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#define SensorPin A1
#define Offset 0
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth 40
int pHArray[ArrayLenth];
int pHArrayIndex=0;
StaticJsonBuffer<200> jsonBuffer; 
SoftwareSerial myserial(7,8);
int RXPin = 2;
int TXPin = 3;
int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Start up the library
  sensors.begin();
  pinMode(LED,OUTPUT);
  myserial.begin(9600);
  DynamicJsonBuffer jsonBuffer;
  
}

void loop() {
  // put your main code here, to run repeatedly:
  ntu's = turbititySensor();
  delay(200);
  temp's = temperature();
  delay(200);
  ph's = phSensor();
  delay(200);
  lat's, lon's = displayInfo();
  delay(200);
  gsmShield(temp's, ph's, ntu's, lat's, lon's);
  delay(1000);
}

float turbititySensor()
{
    volt = 0;
    for(int i=0; i<800; i++)
    {
        volt += ((float)analogRead(sensorPin)/1023)*5;
    }
    volt = volt/800;
    volt = round_to_dp(volt,2);
    if(volt < 2.5){
      ntu = 3000;
    }else{
      ntu = -1120.4*square(volt)+5742.3*volt-4353.8; 
    }
    return(ntu)
    delay(10);
}
 
float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
float temperature()
{
  sensors.requestTemperatures();
  return(sensors.getTempCByIndex(0));
}
float phSensor()
{
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++]=analogRead(SensorPin);
    if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
    voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
    pHValue = 3.5*voltage+Offset;
    samplingTime=millis();
  }
  if(millis() - printTime > printInterval)
  {
    return(pHValue);
    printTime=millis();
  }
}
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min; //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max; //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
void gpsTracker()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();
 
  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while(true);
  }
}
 
float displayInfo()
{
  if (gps.location.isValid())
  {
    return(gps.location.lat(),gps.location.lng());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
}
void gsmShield(float temps, float phs, float nuts, float lats, float longi)
{
 
  if (myserial.available())
  Serial.write(myserial.read());
 
  myserial.println("AT");
  delay(500);
  ShowSerialData();
 
  myserial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  delay(500);
  ShowSerialData();
 
  myserial.println("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"");//APN
  delay(500);
  ShowSerialData();
 
  myserial.println("AT+SAPBR=1,1");
  delay(500);
  ShowSerialData();
 
  myserial.println("AT+SAPBR=2,1");
  delay(500);
  ShowSerialData();
 
 
  myserial.println("AT+HTTPINIT");
  delay(500);
  ShowSerialData();
 
  myserial.println("AT+HTTPPARA=\"CID\",1");
  delay(500);
  ShowSerialData();
 
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& object = jsonBuffer.createObject();
 
  object.set("ph",phs);
  object.set("temp",temps);
  object.set("turbidity",nuts);
  object.set("lattitude",lats);
  object.set("longitude",longi);
 
  object.printTo(Serial);
  Serial.println(" ");  
  String sendtoserver;
  object.prettyPrintTo(sendtoserver);
  delay(500);
 
  myserial.println("AT+HTTPPARA=\"URL\",\"https://node-fake-api-server.herokuapp.com"); //Server address
  delay(4000);
  ShowSerialData();
 
  myserial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(500);
  ShowSerialData();
 
 
  myserial.println("AT+HTTPDATA=" + String(sendtoserver.length()) + ",100000");
  Serial.println(sendtoserver);
  delay(500);
  ShowSerialData();
 
  myserial.println(sendtoserver);
  delay(500);
  ShowSerialData;
 
  myserial.println("AT+HTTPACTION=1");
  delay(500);
  ShowSerialData();
 
  myserial.println("AT+HTTPREAD");
  delay(500);
  ShowSerialData();
 
  myserial.println("AT+HTTPTERM");
  delay(500);
  ShowSerialData;
 
  /********************GSM Communication Stops********************/
 
}
 
 
void ShowSerialData()
{
  while (myserial.available() != 0)
    Serial.write(myserial.read());
  delay(500);
 
}
