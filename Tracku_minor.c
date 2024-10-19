#include <DHT.h>
#include <common.h>
#include <Firebase.h>
#include <FirebaseFS.h>
#include <Utils.h>



//-----------------------------------------------------------------------------------
//FirebaseESP8266.h must be included before ESP8266WiFi.h
#include <FirebaseESP8266.h>  //https://github.com/mobizt/Firebase-ESP8266
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus

//Install ArduinoJson Library
//Note: The latest JSON library might not work with the code. 
//So you may need to downgrade the library to version v5.13.5
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
#define FIREBASE_HOST "***********"
#define FIREBASE_AUTH "*********************"

#define WIFI_SSID "*****************"
#define WIFI_PASSWORD "*************"
//-----------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------
//Define FirebaseESP8266 data object
FirebaseData firebaseData;
FirebaseJson json;
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
//GPS Module RX pin to NodeMCU D1
//GPS Module TX pin to NodeMCU D2
const int RXPin = 4, TXPin = 5;
SoftwareSerial neo6m(RXPin, TXPin);
TinyGPSPlus gps;
//-----------------------------------------------------------------------------------

#define DHTPIN D7                                           // Digital pin connected to DHT11
#define DHTTYPE DHT11                                        // Initialize dht type as DHT 11
DHT dht(DHTPIN, DHTTYPE); 

int gas_sensor = A0; //MQ135 Sensor pin 
float m = -0.3376; //Slope 
float b = 0.7165; //Y-Intercept 
float R0 = 3.12; //Sensor Resistance in fresh air from previous code


void setup()
{

  Serial.begin(115200);
  pinMode(gas_sensor, INPUT); //Set gas sensor as input
  
  neo6m.begin(9600);
  dht.begin();
  wifiConnect();

  Serial.println("Connecting Firebase.....");
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase OK.");

}


void loop() {
  dhtsensorUpdate();
  mq135sensorUpdate();

  smartdelay_gps(1000);

  if(gps.location.isValid()) 
  {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    
    //-------------------------------------------------------------
    //Send to Serial Monitor for Debugging
    //Serial.print("LAT:  ");
    //Serial.println(latitude);  // float to x decimal places
    //Serial.print("LONG: ");
    //Serial.println(longitude);
    //-------------------------------------------------------------
    
    //-------------------------------------------------------------
    if(Firebase.setFloat(firebaseData, "/GPS/f_latitude", latitude))
      {print_ok();}
    else
      {print_fail();}
    //-------------------------------------------------------------
    if(Firebase.setFloat(firebaseData, "/GPS/f_longitude", longitude))
      {print_ok();}
    else
      {print_fail();}
   //-------------------------------------------------------------
   
  }
  else
  {
    Serial.println("No valid GPS data found.");
  }
  
  delay(5000);
  
}


void mq135sensorUpdate(){

  float sensor_volt; //Define variable for sensor voltage 
  float RS_gas; //Define variable for sensor resistance  
  float ratio; //Define variable for ratio
  int sensorValue = analogRead(gas_sensor);
  Serial.print("SENSOR RAW VALUE = ");
  Serial.println(sensorValue);
  if(isnan(sensorValue))
    {
    Serial.println(F("Failed to read from MQ135 sensor!"));
    return;
    }
  sensor_volt = sensorValue*(5.0/1023.0); //Convert analog values to voltage 
  
  RS_gas = ((5.0*10.0)/sensor_volt)-10.0; //Get value of RS in a gas
  
  ratio = RS_gas/R0;  // Get ratio RS_gas/RS_air
  
  
  float ppm_log = (log10(ratio)-b)/m; //Get ppm value in linear scale according to the the ratio value  
  float ppm = pow(10, ppm_log); //Convert ppm value to log scale 
  Serial.print("PPM = ");
  Serial.println(ppm);
  double percentage = ppm/10000; //Convert to percentage 
  Serial.print("Value in Percentage = "); //Load screen buffer with percentage value
  Serial.println(percentage);
  if (Firebase.setFloat(firebaseData, "/FirebaseIOT/AQI", sensorValue))  //pusblish to AQI topic
  {
    Serial.println("Published AQI data");
  }
  else
  {
    Serial.println("Failed to publish AQI data");
  } 
  if (Firebase.setDouble(firebaseData, "/FirebaseIOT/PPM_percentage", percentage))  //publish to ppm topic
  {
    Serial.println("Published ppm data");
  }
  else
  {
    Serial.println("Failed to publish ppm data");
  } 
  
}

void dhtsensorUpdate(){
  
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed 
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(F("Humidity: "));          //printing values on serial monitor
  Serial.print(h);
  Serial.print(F(" Temperature: "));
  Serial.print(t);
  Serial.print(F("C  ,"));
  Serial.print(f);
  Serial.println(F("F  "));

  if (Firebase.setFloat(firebaseData, "/FirebaseIOT/temperature", t))     //publish to temprature topic
  {
    Serial.println("Published temp data");
  }
  else
  {
    Serial.println("Failed to publish temp data");
  }
  if (Firebase.setFloat(firebaseData, "/FirebaseIOT/humidity", h))          //publish to humidity topic
  {
    Serial.println("Published humidity data");
  }
  else
  {
    Serial.println("Failed to publish temp data");
  }
}

static void smartdelay_gps(unsigned long ms)      //function to ensure gps object reads fresh data every cycle
{
  unsigned long start = millis();
  do 
  {
    while (neo6m.available())
      gps.encode(neo6m.read());
  } while (millis() - start < ms);
}


void wifiConnect()                             // wifi connect function
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void print_ok()
{
    Serial.println("------------------------------------");
    Serial.println("OK");
    Serial.println("PATH: " + firebaseData.dataPath());
    Serial.println("TYPE: " + firebaseData.dataType());
    Serial.println("ETag: " + firebaseData.ETag());
    Serial.println("------------------------------------");
    Serial.println();
}

void print_fail()
{
    Serial.println("------------------------------------");
    Serial.println("FAILED");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
}

void firebaseReconnect()          //function to retry firebase connection in case it fails
{
  Serial.println("Trying to reconnect");
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}