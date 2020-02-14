#include <Adafruit_BMP085.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <InfluxDb.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Adafruit_INA219.h>

#define INFLUXDB_HOST ""
const char* ssid     = "";
const char* password = "";

//#define DEBUG
#define ONEWIRE_PIN 23
#define DS18B20_RESOLUTION 10

byte address [8] = {0x28, 0xFF, 0xAB, 0x3D, 0x52, 0x19, 0x01, 0xEA}; // DS18B20 address, outdoor

Adafruit_BMP085 bmp;
WiFiMulti wifimulti;
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);
Influxdb influx(INFLUXDB_HOST);
Adafruit_INA219 ina219_bat1;
Adafruit_INA219 ina219_bat2(0x41); // A0 bridged
Adafruit_INA219 ina219_bat3(0x44); // A1 bridged


unsigned long lastMillis, lastSendMillis, lastDS18B20millis, lastVoltageMillis, lastPrint;
unsigned int bmpReadInterval = 9000;
unsigned int DS18B20readInterval = 9000;
unsigned int sendInterval = 10000;
const int meters = 258;
const int led_pin = 33; 
unsigned int status;
float Vbat_1, Vbat_2, Vbat_3 = 0.0; // Measured battery voltage
int lastVoltMillis = 1000; // voltage measurment interval
float temperature, temperatureDS18B20, pressure, sealevelPressure;
unsigned int OneWireDevices;
int rssi;

void setup() {
    Serial.begin(115200);
    while(!Serial);    // time to get serial running
    Serial.println(F("Project X"));

   
    sensors.begin();
    OneWireDevices = sensors.getDeviceCount();
    status = bmp.begin(); 
    ina219_bat1.begin();
    ina219_bat2.begin();
    ina219_bat3.begin();
   
    sensors.setResolution(address, DS18B20_RESOLUTION);
    pinMode(led_pin, OUTPUT);

    switch (status)
    {
    case 1:
        Serial.println("BMP180 initialization successful");
        break;
    case 0:
    Serial.println("Could not find a valid BMP180 sensor, check wiring, address, sensor ID!");
    break;
    }
       
        delay(1000);
        
         
     Serial.print("Connecting to ");
    Serial.println(ssid);

    wifimulti.addAP(ssid, password);

    if(wifimulti.run() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }

    influx.setPort(9999);
    influx.setVersion(2);
    influx.setOrg("");
    influx.setBucket("");
    influx.setToken("");
    
    Serial.println("-- End of setup --");

    Serial.println();
     Serial.print("DS18B20 resolution: ");
     Serial.println(sensors.getResolution(address));

    delay(5000);
}

void loop() { 
    getbmpData();
    getDS18B20data();
    measureVolt();
    digitalWrite(led_pin, LOW);
        
    if (millis() - lastSendMillis >= sendInterval)
{
    lastSendMillis = millis();
    digitalWrite(led_pin, HIGH);
 InfluxData row("data");
    row.addTag("device", "esp32");
    row.addValue("temperature", temperature);
    row.addValue("pressure", pressure);
    row.addValue("sealevelPressure", sealevelPressure);
    row.addValue("DS18B20", temperatureDS18B20);
    row.addValue("Vbat_1", Vbat_1);
    row.addValue("Vbat_2", Vbat_2);
    row.addValue("Vbat_3", Vbat_3);

    influx.write(row);
    
}



#ifdef DEBUG
if ( millis() - lastPrint >= 5000)
{
    lastPrint = millis();
    Serial.print("----DEBUG----");
    Serial.println();

    Serial.println("----> DS18B20 <----");

    Serial.println();

    Serial.print("Nubmer of devices on bus: "); Serial.println(sensors.getDeviceCount());
    Serial.print("DS18B20 mode:"); Serial.println(sensors.isParasitePowerMode());
    Serial.print("Sensor resolution: "); Serial.println(sensors.getResolution());
    Serial.print("Temperature: "); Serial.print(temperatureDS18B20); Serial.println(" *C"); 

    Serial.println("----> BMP180 <----");

        Serial.println();
        Serial.print("Temperature = ");
        Serial.print(bmp.readTemperature());
        Serial.println(" *C");

        Serial.print("Pressure = ");

        Serial.print(bmp.readPressure() / 100.0F);
        Serial.println(" hPa");

        Serial.print("seaLevelPressure = ");
        Serial.print(bmp.readSealevelPressure(meters)/ 100.0F);
        Serial.println(" hPa");

    Serial.println();

    Serial.println("----> Battery Voltage <----");

    Serial.println();
    Serial.println("Battery 1: ");
    Serial.print("Battery 1 Voltage: "); Serial.print(Vbat_1); Serial.println(" V");
    Serial.println();
    Serial.println("Battery 2: ");
    Serial.print("Battery 2 Voltage: "); Serial.print(Vbat_2); Serial.println(" V");
    Serial.println();
    Serial.println("Battery 3: ");
    Serial.print("Battery 3 Voltage: "); Serial.print(Vbat_3); Serial.println(" V");

    Serial.println();
    Serial.print("Millis: "); Serial.print(millis()/ 1000); Serial.println(" s");
    Serial.print("WiFi signal strenght: "); Serial.println(wifimulti.RSSI());
    
}
#endif

}

void getbmpData() {
    if (millis() - lastMillis >= bmpReadInterval)
    {
        lastMillis = millis();

        temperature = bmp.readTemperature();
        pressure = (bmp.readPressure() / 100);       
        sealevelPressure = (bmp.readSealevelPressure(meters)/ 100);
    }
}

void getDS18B20data(){
    if (millis() - lastDS18B20millis >= DS18B20readInterval)
    {
        lastDS18B20millis = millis();
       sensors.requestTemperatures();
       temperatureDS18B20 = sensors.getTempC(address);
    }
}

void measureVolt(){

  if(millis() - lastVoltMillis >= 1000){
    lastVoltMillis = millis();
  Vbat_1 =  ina219_bat1.getBusVoltage_V();
  Vbat_2 =  ina219_bat2.getBusVoltage_V();
  Vbat_3 =  ina219_bat3.getBusVoltage_V();
    }
}

