/* 
 ESP8266 BlinkWithoutDelay by Simon Peter
 Blink the blue LED on the ESP-01 module
 Based on the Arduino Blink without Delay example
 This example code is in the public domain
 
 The blue LED on the ESP-01 module is connected to GPIO1 
 (which is also the TXD pin; so we cannot use Serial.print() at the same time)
 
 Note that this sketch uses LED_BUILTIN to find the pin with the internal LED
*/
#define DELAY 55
#define DELAY2 100

int ledState = LOW;     
uint8_t puertos[] = {2,0,4,5,14,16}; 
int sisas = 6;
unsigned long previousMillis = 0;
const long interval = 1000;


void setup() {
   for(int i = 0; i < sisas; i++){
    pinMode(puertos[i], OUTPUT);     // Initialize the LED_BUILTIN pin as an output
    digitalWrite(puertos[i],LOW);
  }
  delay(DELAY);
  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[i],HIGH);
  }
}

void loop()
{
 for(int j = 0; j < 3;j++){
  for(int i = 0; i < sisas - 1; i++){
      digitalWrite(puertos[i],LOW);
      delay(DELAY);
      digitalWrite(puertos[i],HIGH);
      delay(DELAY);
  }

  //delay(2000);
  
  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[sisas-i-1],LOW);
      delay(DELAY);
      digitalWrite(puertos[sisas-i-1],HIGH);
      delay(DELAY);
  }
 }

 for(int j = 0; j < 3;j++){
  for(int i = 0; i < sisas - 2; i++){
      digitalWrite(puertos[i],LOW);
      digitalWrite(puertos[i+1],LOW);
      delay(DELAY);
      digitalWrite(puertos[i],HIGH);
      //digitalWrite(puertos[i+1],HIGH);
      delay(DELAY);
  }
  digitalWrite(puertos[sisas-1], HIGH);
  //delay(2000);
  
  for(int i = 0; i < sisas-1; i++){
      digitalWrite(puertos[sisas-i-1],LOW);
      digitalWrite(puertos[sisas-i-2],LOW);
      delay(DELAY);
      digitalWrite(puertos[sisas-i-1],HIGH);
      //digitalWrite(puertos[sisas-i-2],HIGH);
      delay(DELAY);
  }
 }

for(int j = 0; j < 3;j++){
  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[i],LOW);
      delay(DELAY);
  }

  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[sisas-i-1],HIGH);
      delay(DELAY);
  }
//}

//for(int j = 0; j < 3;j++){
  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[sisas-i-1],LOW);
      delay(DELAY);
  }

  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[i],HIGH);
      delay(DELAY);
  }
}
for(int j = 0; j < 3;j++){
  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[i],LOW);
      delay(DELAY);
  }

  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[i],HIGH);
      delay(DELAY);
  }
//}
//for(int j = 0; j < 3;j++){
  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[sisas-i-1],LOW);
      delay(DELAY);
  }

  for(int i = 0; i < sisas; i++){
      digitalWrite(puertos[sisas-i-1],HIGH);
      delay(DELAY);
  }
}

for(int j=0; j < 3; j++){
  for(int i=0; i < sisas; i++){
    digitalWrite(puertos[i],LOW);
    digitalWrite(puertos[sisas-i-1],LOW);
    delay(DELAY);
    digitalWrite(puertos[i],HIGH);
    digitalWrite(puertos[sisas-i-1],HIGH);
  }
}

for(int j=0; j < 3; j++){
  for(int i=2; i >= 0; i--){
    digitalWrite(puertos[i],LOW);
    digitalWrite(puertos[sisas-i-1],LOW);
    delay(150);
    digitalWrite(puertos[i],HIGH);
    digitalWrite(puertos[sisas-i-1],HIGH);
  }
}

for(int j=0; j < 3; j++){
  for(int i=0; i < 3; i++){
    digitalWrite(puertos[i],LOW);
    delay(DELAY2);
    digitalWrite(puertos[i],HIGH);
    delay(DELAY2);
    digitalWrite(puertos[sisas-i-1],LOW);
    delay(DELAY2);
    digitalWrite(puertos[sisas-i-1],HIGH);
    delay(DELAY2);
  }
}
delay(200);
for(int j=0; j < 3; j++){
  for(int i=0; i < 3; i++){
    digitalWrite(puertos[sisas-i-1],LOW);
    delay(DELAY2);
    digitalWrite(puertos[sisas-i-1],HIGH);
    delay(DELAY2);
    digitalWrite(puertos[i],LOW);
    delay(DELAY2);
    digitalWrite(puertos[i],HIGH);
    delay(DELAY2);
  }
}
delay(200);
for(int j=0; j < 3; j++){
  for(int i=0; i < 3; i++){
    digitalWrite(puertos[3+i],LOW);
    digitalWrite(puertos[i],LOW);
    delay(DELAY2);
    digitalWrite(puertos[3+i],HIGH);
    digitalWrite(puertos[i],HIGH);
    delay(DELAY2);
  }
}
delay(200);
for(int j=0; j < 3; j++){
  for(int i=0; i < 3; i++){
    digitalWrite(puertos[sisas-3-i],LOW);
    digitalWrite(puertos[sisas-i],LOW);
    delay(DELAY2);
    digitalWrite(puertos[sisas-3-i],HIGH);
    digitalWrite(puertos[sisas-i],HIGH);
    delay(DELAY2);
  }
}
delay(200);
for(int j=0; j < 3; j++){
  for(int i=0; i < 3; i++){
    digitalWrite(puertos[3+i],LOW);
    digitalWrite(puertos[i],LOW);
    delay(DELAY2);
    digitalWrite(puertos[3+i],HIGH);
    digitalWrite(puertos[i],HIGH);
    delay(DELAY2);
  }

  for(int i=0; i < 3; i++){
    digitalWrite(puertos[sisas-3-i],LOW);
    digitalWrite(puertos[sisas-i],LOW);
    delay(DELAY2);
    digitalWrite(puertos[sisas-3-i],HIGH);
    digitalWrite(puertos[sisas-i],HIGH);
    delay(DELAY2);
  }
}

}
