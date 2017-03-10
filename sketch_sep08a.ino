
// Expose Espressif SDK functionality - wrapped in ifdef so that it still
// compiles on other platforms
#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif
int i=0;
long total = 0;
long tim = 0;
long total2 = 0;
long tim2 = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  #ifdef ESP8266
    uint32_t free = system_get_free_heap_size();
    uint32_t id = system_get_chip_id();
    uint16_t adc_addr[100];
    uint16_t adc_addr2[100];
    uint16_t adc_num = 2;
    uint8_t adc_clk_div = 8;
    //Serial.println(system_get_sdk_version());
    long inicio = micros();
    system_adc_read_fast(adc_addr, adc_num, adc_clk_div);
    long fin1 = micros();
    for(int j = 0; j < adc_num; j++){
      adc_addr2[j] = system_adc_read();
    }
    long fin2 = micros();
    int tot = fin2 - fin1;
    int tot2 = fin1 - inicio;
    tim += tot;
    tim2 += tot2;
    total += 2000000.0/tot;
    total2 += 2000000.0/tot2;
    i++;
    if(i == 100){
      Serial.print("Sampling rate Slow: ");
      Serial.println(total/100);
      Serial.print("Se demoró slow: ");
      Serial.println(tim/100);
      Serial.print("Sampling rate fast: ");
      Serial.println(total2/100);
      Serial.print("Se demoró fast: ");
      Serial.println(tim2/100);
      i = 0;
      tim = 0;
      total = 0;
      tim2 = 0;
      total2 = 0;
    }
  #endif

}
