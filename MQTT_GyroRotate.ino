#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
Madgwick MadgwickFilter;
// BMX055　加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)
/*謎のヘッダーファイル(中身見てない)*/
#include "CDebounce.h"

// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// Global Definition
// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// ========================================
// Wi-Fi setting
// 接続先のSSIDとパスワード
const char* wifi_ssid = "********";
const char* wifi_password = "********";

// ========================================
// MQTT setting
// shiftr.io サイトの情報、Token は自分で作成した Channel から取得
const char* mqttServer   = "broker.shiftr.io";
const int   mqttPort     = 1883;
const char* mqttUser     = "********";
const char* mqttPassword = "********";

// ========================================
// MQTT setting
// 自分で設定するデバイス名
const char* mqttDeviceId = "three_eight_one_tech";

// MQTT Topic 名
const char* mqttTopic_publish   = "********";
const char* mqttTopic_subscribe = "********";

//Connect WiFi Client and MQTT(PubSub) Client
WiFiClient espClient;
PubSubClient g_mqtt_client(espClient);

// ========================================
// JSON setting
// Reserved memory for deserialization
//   enough space for 1 object with 10 members
const int json_object_capacity = 1 * JSON_OBJECT_SIZE(10);

// ========================================
// ESP32 pin configuration
// pin number definition
const int SW2     = 16; // push button
const int G_LED   = 14; // blue LED



/*ジャイロ関連*/
/*3軸加速度データ格納*/
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
/*3軸角速度データ格納*/
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
/*地磁気データ格納*/
int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;
/*MagdwickFilter通過後*/
float roll = 0;
float pitch = 0;
float yaw = 0;

//=====================================================================================//
void BMX055_Init(){
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03);   // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Accl(){
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)  xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)  yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)  zAccl -= 4096;
  xAccl = xAccl * 0.0098; // renge +-2g
  yAccl = yAccl * 0.0098; // renge +-2g
  zAccl = zAccl * 0.0098; // renge +-2g
}
//=====================================================================================//
void BMX055_Gyro(){
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)  xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)  yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag(){
  int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] <<8) | (data[0]>>3));
  if (xMag > 4095)  xMag -= 8192;
  yMag = ((data[3] <<8) | (data[2]>>3));
  if (yMag > 4095)  yMag -= 8192;
  zMag = ((data[5] <<8) | (data[4]>>3));
  if (zMag > 16383)  zMag -= 32768;
}

//IMUの値を取得する
void readSensor(){
  BMX055_Accl();//BMX055 Acceleration Data
  BMX055_Gyro();//BMX055 Gyro Data
  BMX055_Mag(); //BMX055 Magnetic Data
}

void Filter(){
  //MadgwickFilter.updateIMU(xGyro,yGyro,zGryo,xAccl,yAccl,zAccl);
  MadgwickFilter.update(xGyro,yGyro,zGyro,xAccl,yAccl,zAccl,xMag,yMag,zMag);
  roll  = MadgwickFilter.getRoll();
  pitch = MadgwickFilter.getPitch();
  yaw   = MadgwickFilter.getYaw(); 
}

// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// Status monitor for debug
// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// ========================================
// Show status in loop function
//   This function is designed to be called in loop() function, 
//   to show system status regulary on serial console. 
void show_status_loop()
{
    const unsigned long print_interval = 5000; // [ms] of print interval
    static unsigned long last_print_time = millis(); // last time stamp of print

    // if elapsed time is too short...
    if((millis() - last_print_time) < print_interval){
        return; // quit doing nothing
    }

    // update timestamp
    last_print_time = millis();

    // show Wi-Fi status, if not connected
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi: not connected.");
    }

    // show MQTT status, if not connected
    if(!g_mqtt_client.connected()) {
        Serial.println("MQTT: not connected.");
    }

    return;
}

// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// MQTT
// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// ========================================
// Connect to MQTT broker
void connect_to_MQTT_broker()
{
    // Attempt to connect
    if (g_mqtt_client.connect(mqttDeviceId, mqttUser, mqttPassword)) {
        // subscribe here
        g_mqtt_client.subscribe(mqttTopic_subscribe);
        Serial.print("MQTT: Subscribed to topic: ");
        Serial.println(mqttTopic_subscribe);
    } 

    // Show info.
    if(g_mqtt_client.connected()){
        Serial.println("MQTT: Connected.");
    }
}

// ========================================
// MQTT callback on message arrival
//   Note! payload may not be null-terminated.
void MQTT_callback(char* topic, byte* payload, unsigned int length) 
{
	// print message
	Serial.print("MQTT: Received topic = "); 
	Serial.print(topic); 
	Serial.print(", payload = ");
	Serial.write(payload, length); // using write() here, since payload may not be null-terminated
  Serial.println();

    // // Deserialize (or parse) JSON
    // StaticJsonDocument<json_object_capacity> json_doc;
    // DeserializationError err = deserializeJson( json_doc, (char*)payload, length );
    // if(err){
    //     Serial.print(F("deserializeJson() failed with code "));
    //     Serial.println(err.c_str());
    //      return;
    // }

    // // Find element in JSON
    // String color = json_doc["LED"];
    // Serial.println(color);
    // if( color == "RED" ){
    //     Serial.println("RED!!");
    // }
    // // ...
}

void publish_angle_state(float x,float y,float z){

    StaticJsonDocument<json_object_capacity> json_doc;
    json_doc["deviceName"] = mqttDeviceId;
    json_doc["pitch"] = x;
    json_doc["roll"] = y;
    json_doc["yaw"] = z;


    String payload;
    serializeJson( json_doc, payload );
    g_mqtt_client.publish(mqttTopic_publish, payload.c_str());

    // print message
    Serial.print("MQTT: Publish to ");
    Serial.print(mqttTopic_publish);
    Serial.print(", payload = ");
    Serial.println(payload);


}

// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// Push Button
// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// ========================================
// Push-button callback functions
void callback_sw2()
{
    Serial.println("SW2 pushed.");
    digitalWrite(G_LED,HIGH);
    publish_angle_state(pitch,roll,yaw);
    delay(500);
    digitalWrote(G_LED,LOW);
}

// ========================================
// Assign input pin to its callback funciton, with debouncing
CDebounce cSW2_deb(SW2_PIN, callback_sw2, FALLING);

// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// Arduino
// _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
// ========================================
// Setup and loop
void setup()
{
    Serial.begin(9600);

    pinMode(GREEN_PIN, OUTPUT);
    pinMode(SW2_PIN, INPUT);

    // connect to WiFi
    Serial.print("WiFi: SSID: ");
    Serial.println(wifi_ssid);
    WiFi.begin(wifi_ssid, wifi_password);

    // set MQTT broker and callback
    Serial.print("MQTT: Broker: ");
    Serial.println(mqttServer);
    g_mqtt_client.setServer(mqttServer, mqttPort);
    g_mqtt_client.setCallback(MQTT_callback);

    Wire.begin();             //I2CStart
    BMX055_Init();            //BMX055Initialize
    delay(300);
    MadgwickFilter.begin(10); //MadgwickFilter:Sampling rate 10Hz -> Rateと制御周期が噛み合う必要有

}

void loop()
{
    // MQTT broker task
    if(!g_mqtt_client.connected()){
        connect_to_MQTT_broker();
    } else {
        g_mqtt_client.loop();
    }

    // button check... event detection makes callback function call. 
    cSW2_deb.check();

    // show status on console
    show_status_loop();

    //ジャイロデータ取得とMadgwickフィルタ通過
    readSensor(); //Get sensor data
    Filter();     //MadgwickFilter
}

