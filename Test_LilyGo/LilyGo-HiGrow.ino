#include <algorithm>
#include <iostream>
#include <WiFi.h>
#include <Button2.h>            //https://github.com/LennartHennigs/Button2
#include <BH1750.h>             //https://github.com/claws/BH1750
#include <DHT12.h>              //https://github.com/xreef/DHT12_sensor_library
#include "ds18b20.h"
#include "configuration.h"

#ifdef __HAS_BME280__
#include <Adafruit_BME280.h>
#endif

#ifdef __HAS_SHT3X__
#include "SHT3X.h"
#endif

#ifdef __HAS_MOTOR__
#include <Adafruit_NeoPixel.h>
#endif


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
        Serial.println("V");
        Serial.println();
      }
    break;
    default:
        break;
    }
    return true;
}


void setup()
{
    Serial.begin(115200);

    button.setLongClickHandler(smartConfigStart);
    useButton.setLongClickHandler(sleepHandler);

    //! Sensor power control pin , use deteced must set high
    pinMode(POWER_CTRL, OUTPUT);
    digitalWrite(POWER_CTRL, 1);
    delay(1000);

    Wire.begin(I2C_SDA, I2C_SCL);

    deviceProbe(Wire);

    dht12.begin();

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


void loop()
{
    button.loop();
    useButton.loop();

    if (millis() - timestamp > 1000) {
        timestamp = millis();
    higrow_sensors_event_t val = {0};

    get_higrow_sensors_event(BHT1750_SENSOR_ID, val);
    get_higrow_sensors_event(SOIL_SENSOR_ID, val);
    get_higrow_sensors_event(SALT_SENSOR_ID, val);
    get_higrow_sensors_event(VOLTAGE_SENSOR_ID, val);
    get_higrow_sensors_event(DHTxx_SENSOR_ID, val);

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
