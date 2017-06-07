#include <DHT_U.h>
#include <DHT.h>

/*
 ESP32 Based automatic watering system
 it controls the moisture of the soil and waters the plant if the humidity
 is below a critical threshold; it reports the measured value to MQTT broker
 it also read tempreature and humidity from a DHT sensor.
 
 The system goes to deep sleep for 30 minutes before repeting the routine.
 
 The scripts water for 15 seconds the plant, then stops and wait before resensing
 the moisture.
*/

//#include "DHT.h"
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>

// WiFi parameters
#define WLAN_SSID       "ssid"
#define WLAN_PASS       "password"
 
// Adafruit IO
//#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVER      "82.227.0.65"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Beliwars"
#define AIO_KEY         "6934d2422ee14c02954dcd0b80ce68b7"
#define DHT_PIN 22
#define DHT_POWER_PIN 23
#define sisas 1
#define DHT_TYPE DHT11   // DHT 22
#define PUMP_PIN        21
#define MOIST_POWER_PIN  19

#define IRRIGATION_TIME        300 // irrigation time in 50 msec unities
#define IRRIGATION_PAUSE_TIME  1000 // irrigation pause time in 50 ms unities
#define MOISTURE_THRESHOLD     40   //Critic Threshold
#define MOISTURE_HYSTERESIS    10   //Critic Threshold
#define SLEEP_TIME             10000000


uint8_t puertos[] = {18};        //Led port array
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

// Create an ESp32 WiFiClient class to connect to the MQTT server.
WiFiClient client;
//SimpleDHT1 dht;
DHT dht(DHT_PIN, DHT_TYPE);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME,AIO_KEY);

#define IRRIGATION_FEED "/feeds/irrigation"

Adafruit_MQTT_Publish irrigation = Adafruit_MQTT_Publish(&mqtt, IRRIGATION_FEED);
uint64_t chipid;
char ssid[10]={0};

void setup() {
  chipid = ESP.getEfuseMac();
  sprintf(ssid, "%04X%08X",(uint16_t)(chipid>>32),(uint32_t)chipid);
  //ssid = String((uint16_t)(chipid>>32),HEX) + String((uint32_t)chipid,HEX);  
  go2Sleep = false;
  
  Serial.begin(115200);
  
  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
  Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.
  //ssid.toUpperCase();
  Serial.println(ssid);
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
  int cnt=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
    if(cnt++ > 20){
      Serial.println(WiFi.status());
      WiFi.disconnect();
      delay(2000);
      Serial.println("Unable to Connect, trying again from start");
      WiFi.begin(WLAN_SSID, WLAN_PASS);
      cnt=0;
    }
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
  uint32_t freeheap = esp_get_free_heap_size();
  if(go2Sleep){
    Serial.println("Going to DeepSleep zzzzzzz");
    
    StaticJsonBuffer<200> SleepBuffer;
    JsonObject& json1 = SleepBuffer.createObject();
    json1["id"] = ssid;
    json1["heap"] = esp_get_free_heap_size();
    json1["status"] = "Sleep";
    json1.prettyPrintTo(Serial);
    
    char json2[200];
    json1.printTo(json2, sizeof(json2));

    if(!irrigation.publish(json2)){
        Serial.println(F("Failed to publish Sleep phase"));
        if(!mqtt.connected())
          connect();
      }
      //ESP.deepSleep
      WiFi.disconnect();
      esp_deep_sleep(SLEEP_TIME);
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
      moist = 100.0 * (1.0 - moistADC/4095.0);
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
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
      root["id"] = ssid;
      root["heap"] = freeheap;
      root["moist"] = moist;
      root["temp"] = t;
      root["hum"] = h;
      root["status"] = is_pump_on;
      //root["time"] = 1351824120;
      root.prettyPrintTo(Serial);
      
      char json[200];
      root.printTo(json, sizeof(json));

      if(!irrigation.publish(json)){
        Serial.println(F("Failed to publish irrigation JSON"));
        if(!mqtt.connected())
          connect();
      }

      //previous_millis = millis();
      pump_counter = 0;
    }
    
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
  Serial.print(F("Connecting to HOMECENTRAL Broker... "));

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
  Serial.println(F("HOMECENTRAL MQTT Connected!"));
}
