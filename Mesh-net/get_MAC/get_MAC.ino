#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif

void setup(){
  Serial.begin(115200);

}
 
void loop(){
    delay(200);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
 
}
