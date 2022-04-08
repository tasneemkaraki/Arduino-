#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif
#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <MAX30100_PulseOximeter.h>
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
// firebase config
#define FIREBASE_HOST "test-7b7d3-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH_KEY "jIpn8VRMaDDzkHCP8IFTCIFNwwtYLie9fYX7CC0h"
#define I2C_SDA 21
#define I2C_SCL 22
#define REPORTING_PERIOD_MS 500
uint8_t max30100_address = 0x57;
uint8_t irmlx90614_address = 0x5A;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
uint32_t tsLastReport = 0;
PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int inPin = 2 ; int val = 0 ;
long int period = 20000;
unsigned long time_now = 0;
unsigned long timee = 0;
unsigned long timeee = 0;
int ct=0;
int count=0;
int VALUE;

float lat1 , lng1 ;
float temp,p_temp;
//Define the Firebase Data object
FirebaseData firebaseData;
// Define the FirebaseAuth for authentication data
FirebaseAuth auth;
// Define the FirebaseConfig for config data
FirebaseConfig config;
// define beam detection flag
volatile boolean heartBeatDetected = false;
// define beam detection callabck
void onBeatDetected()
{
heartBeatDetected = true;
Serial.println("Beat!");
}
void setup() {
Serial.begin(115200);
Serial2.begin(GPSBaud);
pinMode(inPin, INPUT);
ledcSetup(0,1000,12);
ledcAttachPin(12,0);

// start by connecting to ur AP
WiFi.begin (WIFI_SSID, WIFI_PASSWORD);
// log connecting
Serial.print("Connecting to Wi-Fi");
// inf loop untill connected
while (WiFi.status() != WL_CONNECTED){
Serial.print(".");
delay(300);
}
// make sure you have a connection
Serial.println();
Serial.print("Connected with IP: ");
Serial.println(WiFi.localIP());
Serial.println();
// assign DB meta config data
// @see : https://github.com/mobizt/FirebaseESP32/tree/master/examples/Authentications
config.database_url = FIREBASE_HOST;
config.signer.tokens.legacy_token = FIREBASE_AUTH_KEY;
// keep firebase reconnected
Firebase.reconnectWiFi(true);
// init
Firebase.begin(&config, &auth);
// legacy init
// Firebase.begin(FIREBASE_HOST,FIREBASE_AUTH_KEY);
// log failure in Firebase connection
// begin sire
Wire.begin();
mlx.begin();
// make SURE POX is connected
if (!pox.begin()) {
Serial.println("FAILED");
for(;;); // inf loop untill connection
} else {
Serial.println("SUCCESS");
}
// set POX callback for beam detection
pox.setOnBeatDetectedCallback(onBeatDetected);

}
void upload(){
int heartemergency=0;  
float bpm = pox.getHeartRate()+30;
float spo2 =pox.getSpO2();
if( heartBeatDetected && bpm != 0 && spo2 > 0 ) {
// send beats data
Firebase.setFloat(firebaseData, "/user/HeartRate", bpm);
// log beat data
Serial.print("Heart rate:");
Serial.println(bpm);
Firebase.setFloat(firebaseData, "/user/O2", spo2);
Firebase.setFloat(firebaseData, "/user/heartemergency", heartemergency);

// log so2
Serial.print("bpm / SpO2:");
Serial.print(spo2);
Serial.println("%");



}
printTemp();
gpss();
button();
}
void printTemp(){

int tempemergency=0;

if(millis() >= timeee + 2500 ){
  temp = mlx.readObjectTempC()+1.1;
 if( !isnan(temp) ){
p_temp=temp;
if(temp >= 30 && temp <=40){
//Serial.print("Object = ");
//Serial.print(temp);
 //Serial.println("*C");
Firebase.setFloat(firebaseData, "/user/TEMPERATURE", temp);
ct=0;
    Firebase.setFloat(firebaseData, "/user/tempemergency", tempemergency);

  }
 else  {
    ct++;
    if(ct=5){
    tempemergency=1;
    Firebase.setFloat(firebaseData, "/user/TEMPERATURE", temp);
    Firebase.setFloat(firebaseData, "/user/tempemergency", tempemergency);
    ct=0;

    }


    }

 }
else
{
 //Serial.print(" Object = "); 
 Serial.println(p_temp);
Firebase.setFloat(firebaseData, "/user/TEMPERATURE",p_temp);
  }
  timeee += 2500;

}
  
  
  
  
}

void button(){
val = digitalRead(inPin);
if (val == HIGH) { // check if the input is HIGH (button released)
VALUE = 1 ;
while(millis() <= time_now + period ){
uint8_t octave = 1;
ledcWriteNote(0,NOTE_C,octave);
//Serial.println("1");
Firebase.setInt(firebaseData, "/user/Emergency", VALUE);
}
time_now += period;
}
else {
VALUE = 0 ;
ledcWriteTone(0,2000);
//Serial.println("0");
Firebase.setInt(firebaseData, "/user/Emergency", VALUE);
}
}

void gpss(){
lat1 = 31.50729737386112;
lng1 = 35.09072365142806;
Firebase.setFloat(firebaseData, "/user/Latitude", lat1);
Firebase.setFloat(firebaseData, "/user/Longitude", lng1);

//lat1 = gps.location.lat();
//lng1 = gps.location.lng();
// This sketch displays information every time a new sentence is correctly encoded.
while (Serial2.available() > 0 ){
gps.encode(Serial2.read());
if (gps.location.isUpdated()&& millis() >= timee +3000){
Serial.print("Latitude= ");
Serial.print(gps.location.lat(), 6);
//Firebase.setFloat(firebaseData, "/user/Latitude", lat1);
Serial.print(" Longitude= ");
Serial.println(gps.location.lng(), 6);
//Firebase.setFloat(firebaseData, "/user/Longitude", lng1);
timee += 3000;
}
}
}


void loop(){
pox.update();
if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
pox.shutdown();
upload();
pox.resume();
tsLastReport = millis();
}

}
