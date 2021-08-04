#include <algorithm>
#include <iostream>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Button2.h>            //https://github.com/LennartHennigs/Button2
#include <BH1750.h>             //https://github.com/claws/BH1750
#include <DHT12.h>              //https://github.com/xreef/DHT12_sensor_library
#include "ds18b20.h"
#include "configuration.h"
#include <esp_now.h>

#ifdef __HAS_BME280__
#include <Adafruit_BME280.h>
#endif

#ifdef __HAS_SHT3X__
#include "SHT3X.h"
#endif

#ifdef __HAS_MOTOR__
#include <Adafruit_NeoPixel.h>
#endif
#include <WiFi.h>
#include <FirebaseESP32.h>
 
#define FIREBASE_HOST "espwifi-3f682-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "Ftffa3jEOxhZWYEUM2JvqhgP03kAPnyRKXU1zfo9"

int Vrdata = 0; 
int Vresistor = A0; 
FirebaseJson json1;
FirebaseJson json2;
FirebaseJson json3;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

FirebaseData firebaseData;


typedef struct struct_message {
  int id;
  int value1;
  int value2;
  int value3;
} struct_message;

// Create a struct_message to hold outgoing sensor readings
struct_message sendData;

// Create a struct_message called myData
struct_message myData;

//********************** sensor from here **********************

typedef enum {
    BME280_SENSOR_ID,
    DHTxx_SENSOR_ID,
    SHT3x_SENSOR_ID,
    BHT1750_SENSOR_ID,
    SOIL_SENSOR_ID,
    SALT_SENSOR_ID,
    DS18B20_SENSOR_ID,
    VOLTAGE_SENSOR_ID,
} sensor_id_t;

typedef struct {
    uint32_t timestamp;     /**< time is in milliseconds */
    float temperature;      /**< temperature is in degrees centigrade (Celsius) */
    float light;            /**< light in SI lux units */
    float pressure;         /**< pressure in hectopascal (hPa) */
    float humidity;         /**<  humidity in percent */
    float altitude;         /**<  altitude in m */
    float voltage;           /**< voltage in volts (V) */
    uint8_t soli;           //Percentage of soil
    uint8_t salt;           //Percentage of salt
} higrow_sensors_event_t;


BH1750              lightMeter(OB_BH1750_ADDRESS);  //0x23
DHT12               dht12(DHT12_PIN, true);
Button2             button(BOOT_PIN);
Button2             useButton(USER_BUTTON);

#ifdef __HAS_DS18B20__
DS18B20             dsSensor(DS18B20_PIN);
#endif /*__HAS_DS18B20__*/

#ifdef __HAS_SHT3X__
SHT3X               sht30;
#endif /*__HAS_SHT3X__*/

#ifdef __HAS_BME280__
Adafruit_BME280     bme;                            //0x77
#endif /*__HAS_BME280__*/

#ifdef __HAS_MOTOR__
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
#endif  /*__HAS_MOTOR__*/


bool                has_bmeSensor   = true;
bool                has_lightSensor = true;
bool                has_dhtSensor   = true;
uint64_t            timestamp       = 0;



#ifdef __HAS_BME280__
Card *bmeTemperature    = new Card(&dashboard, TEMPERATURE_CARD, DASH_BME280_TEMPERATURE_STRING, "°C");
Card *bmeHumidity       = new Card(&dashboard, HUMIDITY_CARD,   DASH_BME280_HUMIDITY_STRING, "%");
Card *bmeAltitude       = new Card(&dashboard, GENERIC_CARD,    DASH_BME280_ALTITUDE_STRING, "m");
Card *bmePressure       = new Card(&dashboard, GENERIC_CARD,    DASH_BME280_PRESSURE_STRING, "hPa");
#endif  /*__HAS_BME280__*/

#ifdef __HAS_DS18B20__
Card *dsTemperature     = new Card(&dashboard, TEMPERATURE_CARD, DASH_DS18B20_STRING, "°C");
#endif  /*__HAS_DS18B20__*/

#ifdef __HAS_SHT3X__
Card *sht3xTemperature    = new Card(&dashboard, TEMPERATURE_CARD, DASH_SHT3X_TEMPERATURE_STRING, "°C");
Card *sht3xHumidity       = new Card(&dashboard, HUMIDITY_CARD, DASH_SHT3X_HUMIDITY_STRING, "%");
#endif  /*__HAS_DS18B20__*/

#ifdef __HAS_MOTOR__
Card motorButton(&dashboard, BUTTON_CARD, DASH_MOTOR_CTRL_STRING);
#endif  /*__HAS_MOTOR__*/


void deviceProbe(TwoWire &t);

void smartConfigStart(Button2 &b)
{
    Serial.println("smartConfigStart...");
    WiFi.disconnect();
    WiFi.beginSmartConfig();
    while (!WiFi.smartConfigDone()) {
        Serial.print(".");
        delay(200);
    }
    WiFi.stopSmartConfig();
    Serial.println();
    Serial.print("smartConfigStop Connected:");
    Serial.print(WiFi.SSID());
    Serial.print("PSW: ");
    Serial.println(WiFi.psk());
}

void sleepHandler(Button2 &b)
{
    Serial.println("Enter Deepsleep ...");
    esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
    delay(1000);
    esp_deep_sleep_start();
}



bool get_higrow_sensors_event(sensor_id_t id, higrow_sensors_event_t &val)
{
    memset(&val, 0, sizeof(higrow_sensors_event_t));
    switch (id) {
#ifdef __HAS_BME280__
    case BME280_SENSOR_ID: {
        if (has_bmeSensor) {
            val.temperature = bme.readTemperature();
            val.humidity = (bme.readPressure() / 100.0F);
            val.altitude = bme.readAltitude(1013.25);
          Serial.print("Temperature BME280: ");
          Serial.print(val.temperature);
          Serial.println("°C");
          Serial.print("Humidity BME280: ");
          Serial.print(val.humidity);
          Serial.println("%");
          Serial.print("altitude BME280: ");
          Serial.print(val.altitude);
          Serial.println("m");
          Serial.println();
        }
    }
    break;
#endif

#ifdef __HAS_SHT3X__
    case SHT3x_SENSOR_ID: {
        if (has_dhtSensor) {
            if (sht30.get()) {
                val.temperature = sht30.cTemp;
                val.humidity = sht30.humidity;
            Serial.print("Temperature SHT3x: ");
            Serial.print(val.temperature);
            Serial.println("°C");
            Serial.print("Humidity SHT3x: ");
            Serial.print(val.humidity);
            Serial.println("%");
            Serial.println();
            }
        }

    }
    break;
#endif

    case DHTxx_SENSOR_ID: {
        val.temperature = dht12.readTemperature();
        val.humidity = dht12.readHumidity();
        if (isnan(val.temperature)) {
            val.temperature = 0.0;
        }
        if (isnan(val.humidity)) {
            val.humidity = 0.0;
        }
    }
    break;
/*
    case BHT1750_SENSOR_ID: {
        if (has_lightSensor) {
            val.light = lightMeter.readLightLevel();
        } else {
            val.light = 0;
        }
            Serial.print("Humidity DHTxx: ");
        Serial.print(val.humidity);
        Serial.println("%");
        Serial.print("Temperature DHTxx: ");
        Serial.print(val.temperature);
        Serial.println("°C");
        Serial.println();
    }
    break;
    */
    
    
    case SOIL_SENSOR_ID: {
        uint16_t soil = analogRead(SOIL_PIN);
        val.soli = map(soil, 0, 4095, 100, 0);
                Serial.print("Soil: ");
        Serial.print(val.soli);
        sendData.value1 = val.soli; 
        Serial.println("%");
        Serial.println();
    }
    break;

    case BHT1750_SENSOR_ID: {
        if (has_lightSensor) {
          val.light = lightMeter.readLightLevel();
        } else {
          val.light = 0;
        }
        Serial.print("Light HT1750: ");
        Serial.print(val.light);
        Serial.println("lux");
        Serial.println();
      }
     break;
    case SALT_SENSOR_ID: {
        uint8_t samples = 120;
        uint32_t humi = 0;
        uint16_t array[120];
        for (int i = 0; i < samples; i++) {
            array[i] = analogRead(SALT_PIN);
            delay(2);
        }
        std::sort(array, array + samples);
        for (int i = 1; i < samples - 1; i++) {
            humi += array[i];
        }
        humi /= samples - 2;
        val.salt = humi;
        Serial.print("Salt: ");
        Serial.print(val.salt);
        sendData.value2 = val.salt;
        Serial.println("%");
        Serial.println();
    }
    break;
#ifdef __HAS_DS18B20__
    case DS18B20_SENSOR_ID: {
        val.temperature = dsSensor.temp();
        if (isnan(val.temperature) || val.temperature > 125.0) {
            val.temperature = 0;
              Serial.print("Temperature DS18B20: ");
          Serial.print(val.temperature);
          Serial.println("°C");
          Serial.println();
        }
    }
#endif
    break;
    case VOLTAGE_SENSOR_ID: {
        int vref = 1100;
        uint16_t volt = analogRead(BAT_ADC);
        val.voltage = ((float)volt / 4095.0) * 6.6 * (vref);
          Serial.print("Voltage: ");
        Serial.print(val.voltage);
        sendData.value3 = val.voltage;
        Serial.println("V");
        Serial.println();
      }
    break;
    default:
        break;
    }
    return true;
}




void deviceProbe(TwoWire &t)
{
    uint8_t err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        t.beginTransmission(addr);
        err = t.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            switch (addr) {
            case OB_BH1750_ADDRESS:
                has_dhtSensor = true;
                break;
            case OB_BME280_ADDRESS:
                has_bmeSensor = true;
                break;
            case OB_SHT3X_ADDRESS:
                has_dhtSensor = true;
                break;
            default:
                break;
            }
            nDevices++;
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}


// *****************  here mesh data  *****************


// REPLACE WITH THE MAC Address of your receiver
uint8_t Address1[] = {0xC4, 0x4F, 0x33, 0x7F, 0xC5, 0x51};
uint8_t Address2[] = {0xC4, 0x4F, 0x33, 0x7F, 0xC0, 0xA1};
uint8_t Address3[] = {0xC4, 0x4F, 0x33, 0x7F, 0xC5, 0x45};
uint8_t Address4[] = {0xC4, 0x4F, 0x33, 0x7F, 0xC3, 0xDD};
uint8_t Address5[] = {0xC4, 0x4F, 0x33, 0x7F, 0xBE, 0x0D};
uint8_t Address6[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address7[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address8[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address9[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address10[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};



// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;
struct_message board4;
struct_message board5;
struct_message board6;
struct_message board7;
struct_message board8;
struct_message board9;
struct_message board10;

// Create an array with all the structures
struct_message boardsStruct[] = {board1, board2, board3, board4, board5, board6, board7, board8, board9, board10};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);

  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].value1 = myData.value1;
  boardsStruct[myData.id - 1].value2 = myData.value2;
  boardsStruct[myData.id - 1].value3 = myData.value3;
  Serial.printf("value1: %d \n", boardsStruct[myData.id - 1].value1);
  Serial.printf("value2: %d \n", boardsStruct[myData.id - 1].value2);
  Serial.printf("value3: %d \n", boardsStruct[myData.id - 1].value3);
  Serial.println();
}



void sendDataToAll()
{
  //!!!ID for each Board!!!
  sendData.id = 4;


  //* How to set Data
  //sendData.value1 = 77;
  //sendData.value2 = 105;
  //sendData.value3 = false;
  //*/

  esp_err_t result = esp_now_send(0, (uint8_t *) &sendData, sizeof(struct_message));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void InitEspNow()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

// conect to wifi AP
  WiFi.begin("tomer","0546428644");
  //check the connection to the internet
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100); 
  }
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
    //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
     Vrdata = analogRead(Vresistor);
 int Sdata = map(Vrdata,0,4095,0,1000);
 Serial.println(Sdata); 
delay(100); 
  int x =6;
  //json1.set("/soil", sendData.value1 );
  //json2.set("/salt", sendData.value2);
  //json3.set("/battery",8 );
  //Firebase.updateNode(firebaseData,"/Id1",json1);
  //Firebase.updateNode(firebaseData,"/Id1",json2);
  //Firebase.updateNode(firebaseData,"/Id1",json3);

  Serial.println(WiFi.localIP());
  
  //connect to firebase
//  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH); // connect to the firebase database




  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // register peer
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // register first peer
  memcpy(peerInfo.peer_addr, Address1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // register second peer
  memcpy(peerInfo.peer_addr, Address2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register third peer
  memcpy(peerInfo.peer_addr, Address3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register fourth peer
  memcpy(peerInfo.peer_addr, Address4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register fifth peer
  memcpy(peerInfo.peer_addr, Address5, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register sixth peer
  memcpy(peerInfo.peer_addr, Address6, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register next peer
  memcpy(peerInfo.peer_addr, Address7, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register next peer
  memcpy(peerInfo.peer_addr, Address8, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register next peer
  memcpy(peerInfo.peer_addr, Address9, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register next peer
  memcpy(peerInfo.peer_addr, Address10, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}


void setup()
{
    Serial.begin(115200);

    InitEspNow();
    button.setLongClickHandler(smartConfigStart);
    useButton.setLongClickHandler(sleepHandler);

    //! Sensor power control pin , use deteced must set high
    pinMode(POWER_CTRL, OUTPUT);
    digitalWrite(POWER_CTRL, 1);
    delay(1000);

    Wire.begin(I2C_SDA, I2C_SCL);

    deviceProbe(Wire);

    dht12.begin();
    timeClient.begin();
    timeClient.setTimeOffset(3600);
    if (!lightMeter.begin()) {
        Serial.println(F("Could not find a valid BH1750 sensor, check wiring!"));
        has_lightSensor = false;
    }

#ifdef __HAS_BME280__
    if (!bme.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        has_bmeSensor = false;
    }
#endif /*__HAS_BME280__*/

#ifdef __HAS_SHT3X__
    Wire1.begin(21, 22);
    deviceProbe(Wire1);
#endif /*__HAS_SHT3X__*/



#ifdef __HAS_MOTOR__

    pixels.begin();
    pixels.setPixelColor(0, 0xFF0000);
    pixels.show();

    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);
   /* motorButton.attachCallback([&](bool value) {
        Serial.println("motorButton Triggered: " + String((value) ? "true" : "false"));
        digitalWrite(MOTOR_PIN, value);
        pixels.setPixelColor(0, value ? 0x00FF00 : 0);
        pixels.show();
        motorButton.update(value);
    });*/
#endif  /*__HAS_MOTOR__*/
 

}

int counter =0;
void loop()
{
    button.loop();
    useButton.loop();
    while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  formattedDate = timeClient.getFormattedDate();
   Serial.println(formattedDate);
   delay(3000);
    if (millis() - timestamp > 1000) {
        timestamp = millis();
    higrow_sensors_event_t val = {0};

    get_higrow_sensors_event(BHT1750_SENSOR_ID, val);
    get_higrow_sensors_event(SOIL_SENSOR_ID, val);
    get_higrow_sensors_event(SALT_SENSOR_ID, val);
    get_higrow_sensors_event(VOLTAGE_SENSOR_ID, val);
    get_higrow_sensors_event(DHTxx_SENSOR_ID, val);
//  char salt[];
//  char battery[];
 //sprintf(str, "%d",counter);
//  snprintf(soil, 200, "/soil/%s", formattedDate);
//  snprintf(salt, 200, "/salt/%s", formattedDate);
//  snprintf(battery, 200, "/battery/%s", formattedDate);
//  strcpy(soil,formattedDate);
  String soil = "/soil/" + formattedDate;
    String salt = "/salt/" + formattedDate;
      String battery = "/battery/" + formattedDate;
  json1.set(soil , sendData.value1 );
  json2.set(salt, sendData.value2);
  json3.set(battery, sendData.value3 );
  Firebase.updateNode(firebaseData,"/Id4",json1);
  Firebase.updateNode(firebaseData,"/Id4",json2);
  Firebase.updateNode(firebaseData,"/Id4",json3);
  counter = counter +1;

formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);

  
#ifdef __HAS_BME280__
        get_higrow_sensors_event(BME280_SENSOR_ID, val);
#endif /*__HAS_BME280__*/

#ifdef __HAS_SHT3X__
        get_higrow_sensors_event(SHT3x_SENSOR_ID, val);
#endif  /*__HAS_SHT3X__*/


#ifdef __HAS_DS18B20__
        get_higrow_sensors_event(DS18B20_SENSOR_ID, val);
#endif  /*__HAS_DS18B20__*/

    }


sendDataToAll();
  delay(2000);

  // Acess the variables for board 1
  //*
    byte value1_board1 = boardsStruct[0].value1;
    int value2_board1 = boardsStruct[0].value2;
    double value3_board1 = boardsStruct[0].value3;
  //*/
  
}
