/*
 ESP8266 Based automatic watering system
 it controls the moisture of the soil and waters the plant if the humidity
 is below a critical threshold; it reports the measured value to MQTT broker
 it also read tempreature and humidity from a DHT sensor.
 
 The system goes to deep sleep for 30 minutes before repeting the routine.
 
 The scripts water for 15 seconds the plant, then stops and wait before resensing
 the moisture.
*/

#include "DHT.h"
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>

// WiFi parameters
#define WLAN_SSID       "ssid"
#define WLAN_PASS       "pass"
 
// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Beliwars"
#define AIO_KEY         "6934d2422ee14c02954dcd0b80ce68b7"
#define DHT_PIN 13
#define DHT_POWER_PIN 12
#define sisas 1
#define DHT_TYPE DHT22   // DHT 22
#define PUMP_PIN        4
#define MOIST_POWER_PIN  5

#define IRRIGATION_TIME        300 // irrigation time in 50 msec unities
#define IRRIGATION_PAUSE_TIME  1000 // irrigation pause time in 50 ms unities
#define MOISTURE_THRESHOLD     40   //Critic Threshold
#define MOISTURE_HYSTERESIS    10   //Critic Threshold
#define SLEEP_TIME             1800000000

// Expose Espressif SDK functionality - wrapped in ifdef so that it still
// compiles on other platforms
#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

uint8_t puertos[] = {14};        //Led port array
//uint8_t sisas = 2;                //number of led to include in Led animation
//unsigned long previous_millis;
unsigned long previous_millis2;   //variables to track loop counts  
uint32_t pump_start = 0;               //counter when pump has started
uint8_t state;                    //tmp state of system
int8_t pt = 0;                    //pt to Led array
uint8_t semaforo = 0;             //flag to control
//uint8_t semafo = 0;
uint8_t seq = 0;                  //flag to switch between sequences
uint32_t pump_counter = 0;        //counter of pump dedicated code
uint8_t led_counter = 0;          //counter of led dedicated code
float h = 0;                    //ambient humidity                  
float t = 0;                    //ambient temperature
float moist = 0;               //soil moisture
bool turn_on = false;
bool is_pump_on = false;
bool go2Sleep;
uint8_t previous_state;


// irrigator state
typedef enum {
  s_idle             = 0,  // irrigation idle
  s_irrigation_start = 1,  // start irrigation
  s_irrigation       = 2,  // irrigate
  s_irrigation_stop  = 3,  // irrigation stop
} e_state;

// Functions
void connect();

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
//SimpleDHT1 dht;
DHT dht(DHT_PIN, DHT_TYPE);
// Store the MQTT server, client ID, username, and password in flash memory.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;

// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] PROGMEM  = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/
const char MOISTURE_FEED[] PROGMEM = AIO_USERNAME "/feeds/Moisture_2";
const char TEMP_FEED[] PROGMEM = AIO_USERNAME "/feeds/Temperature_2";
const char HUMIDITY_FEED[] PROGMEM = AIO_USERNAME "/feeds/Humidity_2";
const char PUMP_FEED[] PROGMEM = AIO_USERNAME "/feeds/pump_status_2";

Adafruit_MQTT_Publish moisture = Adafruit_MQTT_Publish(&mqtt, MOISTURE_FEED);
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, TEMP_FEED);
Adafruit_MQTT_Publish humid = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);
Adafruit_MQTT_Publish pump = Adafruit_MQTT_Publish(&mqtt, PUMP_FEED);

void setup() {
  go2Sleep = false;
  Serial.begin(9600);
  state = s_idle;
  previous_state = s_idle;
  for(int i = 0; i < sisas; i++){
    pinMode(puertos[i], OUTPUT);     // Initialize the LED_BUILTIN pin as an output
    digitalWrite(puertos[i],LOW);
  }
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(MOIST_POWER_PIN, OUTPUT);
  pinMode(DHT_POWER_PIN, OUTPUT);
  digitalWrite(MOIST_POWER_PIN, LOW);
  digitalWrite(DHT_POWER_PIN, LOW);
  
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  // connect to adafruit io
  connect();
  dht.begin();
  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[i],HIGH);
  }
  //previous_millis = millis();
  previous_millis2 = millis();
  //pump_start = millis();
}

// the loop function runs over and over again forever
void loop(){
  if(go2Sleep){
    Serial.println("Going to DeepSleep zzzzzzz");
    if(!pump.publish(3)){
        Serial.println(F("Failed to publish state"));
        if(!mqtt.connected())
          connect();
      }
    ESP.deepSleep(SLEEP_TIME, WAKE_RF_DEFAULT);
    delay(1000);
  }
  
  if(millis() - previous_millis2 > 50){
    
    if(led_counter++ > 4){
      digitalWrite(puertos[pt], !digitalRead(puertos[pt]));  // Turn the LED off by making the voltage HIGH
      if(++pt > sisas  && semaforo == 0){
        pt = 0;
        semaforo = 1;
      }else if(pt > sisas && semaforo == 1){
        seq = 1;
        semaforo = 0;
        pt = 0;
      }
      led_counter = 0;
    }

    if(pump_counter == 50){
      digitalWrite(DHT_POWER_PIN, HIGH);
      //digitalWrite(MOIST_POWER_PIN, HIGH);
    }

    if(pump_counter == 100){
      //dht.read(DHT_PIN, &t, &h, NULL);
      t = dht.readTemperature();
      h = dht.readHumidity();
      Serial.println(h);
      Serial.println(t);
      if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
        t=0;
        h=0;
        //continue;
      }
      digitalWrite(DHT_POWER_PIN, LOW);
    }

    if(pump_counter == 150){
      digitalWrite(MOIST_POWER_PIN, HIGH);
    }
  
    if(pump_counter++ > 200){
      int moistADC = analogRead(A0);
      
      Serial.print("Valor ADC: ");
      Serial.println(moistADC);
      moist = 100.0 * (1.0 - moistADC/1023.0);
      Serial.print("moisture del terreno: ");
      Serial.print(moist, 2);
      Serial.println("%");
      if(moist < MOISTURE_THRESHOLD && !is_pump_on){
        //digitalWrite(13, HIGH);
        turn_on = true;
        Serial.println("turning on en loop principal");
        //state = s_irrigation_start;
      }else if((moist > MOISTURE_THRESHOLD + MOISTURE_HYSTERESIS) && is_pump_on){
        turn_on = false;
        Serial.println("turning off en loop principal");
        previous_state = state;
        state = s_irrigation_stop;
        go2Sleep = true;
      }
      digitalWrite(MOIST_POWER_PIN, LOW);
      /*#ifdef ESP8266
        uint32_t free = system_get_free_heap_size();
        uint32_t id = system_get_chip_id();
      #endif*/
      /*StaticJsonBuffer<200> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
      root["id"] = id;
      root["heap"] = free;
      root["moist"] = moisture;
      root["temp"] = t;
      root["hum"] = h;
      
      char json[200];
      root.printTo(json, sizeof(json));*/
      if(!moisture.publish(moist)){
        Serial.println(F("Failed to publish moist"));
        if(!mqtt.connected())
          connect();
      }

      if(!temp.publish(t)){
        Serial.println(F("Failed to publish temp"));
        if(!mqtt.connected())
          connect();
      }

      if(!humid.publish(h)){
        Serial.println(F("Failed to publish hum"));
        if(!mqtt.connected())
          connect();
      }

      uint8_t pump_state = 0;
      if(is_pump_on)
        pump_state = 1;
      if(!pump.publish(pump_state)){
        Serial.println(F("Failed to publish state"));
        if(!mqtt.connected())
          connect();
      }
      //previous_millis = millis();
      pump_counter = 0;
    }

    /*if(turn_on && !is_pump_on && (millis() - pump_start > IRRIGATION_TIME)){
      delay(1000);
    }*/
    
    // irrigator state machine
   switch(state)
   {
     case s_idle:     
       if (pump_start <= IRRIGATION_PAUSE_TIME){
         pump_start++;
        // Serial.println("incrementando pump_start");
         //Serial.println(pump_start);
       }
       
       if (pump_start >= IRRIGATION_PAUSE_TIME && turn_on && !is_pump_on)
       {
           previous_state = state;
           state = s_irrigation_start;
           Serial.println("pasando a irrigation start");
       }else if(pump_start >= IRRIGATION_PAUSE_TIME){
          pump_start = 0;
          int tmp_thr = MOISTURE_THRESHOLD;
          if(previous_state != s_idle)
            tmp_thr += MOISTURE_HYSTERESIS;
          
          if(moist > tmp_thr)
            go2Sleep = true;
       }
       break;
     case s_irrigation_start:
       pump_start = 0;
       digitalWrite(PUMP_PIN, HIGH);
       is_pump_on = true;
       //esp.send(msgMotorPump.set((uint8_t)1));
       previous_state = state;
       state = s_irrigation;
       Serial.println("prendiendo bombita");
       break;
     case s_irrigation:
         Serial.println(pump_start);
         Serial.println(IRRIGATION_TIME);
       if (pump_start++ > IRRIGATION_TIME){
         previous_state = state;
         state = s_irrigation_stop;
         //Serial.println("pasando a irrigation stop");
       }
       break;
     case s_irrigation_stop:
       pump_start = 0;
       previous_state = state;
       state = s_idle;
       digitalWrite(PUMP_PIN, LOW);
       turn_on = false;
       is_pump_on = false;
       Serial.println("deteniendo bombita");
       break;
   }
    
    previous_millis2 = millis();   
  
  }
}


void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));

  int8_t ret;
  int8_t count = 0;

  while (((ret = mqtt.connect()) != 0) && count++ < 5) {

    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(5000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}
