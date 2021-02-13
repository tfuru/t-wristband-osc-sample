#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>

#include <SparkFunLSM9DS1.h>

#include <WiFiManager.h>

#include <ArduinoOSC.h>

#include "config.h"
#include "ttgo.h"

TFT_eSPI tft = TFT_eSPI();
// PCF8563_Class rtc;
LSM9DS1 imu;
WiFiManager wifiManager;

bool pressed = false;
char buff[256];

const float alpha = 0.8f;
typedef struct {
  int index;
  float ax,ay,az;
  float gx,gy,gz;
  float mx,my,mz;  
} VMT;

VMT vmt;

void configModeCallback (WiFiManager *myWiFiManager) {
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    //if you used auto generated SSID, print it
    Serial.println(myWiFiManager->getConfigPortalSSID());

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Connect hotspot name ",  20, tft.height() / 2 - 20);
    tft.drawString("configure wrist",  35, tft.height() / 2  + 20);
    tft.setTextColor(TFT_GREEN);
    tft.drawString("\"T-Wristband\"",  40, tft.height() / 2 );
}

void initWiFi() {
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setBreakAfterConfig(true);
    wifiManager.autoConnect("T-Wristband");
}

void initTFT() {
    tft.init();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  160, 80, ttgo);
}

void initI2C(){
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
}

uint16_t setupIMU() {
    pinMode(INT2_PIN_DRDY, INPUT);
    pinMode(INT1_PIN_THS, INPUT);
    pinMode(INTM_PIN_THS, INPUT);
    pinMode(RDYM_PIN, INPUT);

    // gyro.latchInterrupt controls the latching of the
    // gyro and accelerometer interrupts (INT1 and INT2).
    // false = no latching
    imu.settings.gyro.latchInterrupt = false;
    // Set gyroscope scale to +/-245 dps:
    imu.settings.gyro.scale = 245;
    // Set gyroscope (and accel) sample rate to 14.9 Hz
    imu.settings.gyro.sampleRate = 1;
    // Set accelerometer scale to +/-2g
    imu.settings.accel.scale = 2;
    // Set magnetometer scale to +/- 4g
    imu.settings.mag.scale = 4;
    // Set magnetometer sample rate to 0.625 Hz
    imu.settings.mag.sampleRate = 0;
    bool r =  imu.begin(LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1), Wire);
    if (r) {
        imu.sleepGyro(false);
        return true;
    }
    return false;
}

void getIMU()
{
    // Update the sensor values whenever new data is available
    if ( imu.gyroAvailable() ) {
        // To read from the gyroscope,  first call the
        // readGyro() function. When it exits, it'll update the
        // gx, gy, and gz variables with the most current data.
        imu.readGyro();
    } else {
        Serial.println("Invalid gyroscope");
    }
    if ( imu.accelAvailable() ) {
        // To read from the accelerometer, first call the
        // readAccel() function. When it exits, it'll update the
        // ax, ay, and az variables with the most current data.
        imu.readAccel();
    } else {
        Serial.println("Invalid accelerometer");
    }
    if ( imu.magAvailable() ) {
        // To read from the magnetometer, first call the
        // readMag() function. When it exits, it'll update the
        // mx, my, and mz variables with the most current data.
        imu.readMag();
    } else {
        Serial.println("Invalid magnetometer");
    }

    // 加速度
    vmt.ax = imu.calcAccel(imu.ax);
    vmt.ay = imu.calcAccel(imu.ay);
    vmt.az = imu.calcAccel(imu.az);

    // ジャイロ
    vmt.gx = imu.calcGyro(imu.gx);
    vmt.gy = imu.calcGyro(imu.gy);
    vmt.gz = imu.calcGyro(imu.gz);

    // 磁力
    vmt.mx = imu.calcMag(imu.mx);
    vmt.my = imu.calcMag(imu.my);
    vmt.mz = imu.calcMag(imu.mz);    
}

void showIMU() {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);

    snprintf(buff, sizeof(buff), "--  ACC  GYR   MAG");
    tft.drawString(buff, 0, 0);
    snprintf(buff, sizeof(buff), "x %.2f  %.2f  %.2f", vmt.ax, vmt.gx, vmt.mx);
    tft.drawString(buff, 0, 16);
    snprintf(buff, sizeof(buff), "y %.2f  %.2f  %.2f", vmt.ay, vmt.gy, vmt.my);
    tft.drawString(buff, 0, 32);
    snprintf(buff, sizeof(buff), "z %.2f  %.2f  %.2f", vmt.az, vmt.gz, vmt.mz);
    tft.drawString(buff, 0, 48);
    snprintf(buff, sizeof(buff), "index %d", vmt.index);
    tft.drawString(buff, 0, 64);
}

void sendIMU() {
    // IMUデータ そのまま OSCで送信する
    // DEVICE ax,ay,az,gx,gy,gz,mx,my,mz
    OscWiFi.send(OSC_HOST,
                 OSC_SEND_PORT,
                 "/DEVICE", 
                    vmt.index,
                    vmt.ax,
                    vmt.ay,
                    vmt.az,
                    vmt.gx,
                    vmt.gy,
                    vmt.gz,
                    vmt.mx,
                    vmt.my,
                    vmt.mz);
}

void setup() {
  Serial.begin(115200);

  pinMode(TP_PIN_PIN, INPUT);
  //! Must be set to pull-up output mode in order to wake up in deep sleep mode
  pinMode(TP_PWR_PIN, PULLUP);
  digitalWrite(TP_PWR_PIN, HIGH);

  initTFT();
  initI2C();

  uint16_t status = setupIMU();
  if (status == false) {
    Serial.print("Failed to connect to IMU: 0x");
    Serial.println(status, HEX);
    while (1) ;
  }

  initWiFi();
}

void loop() {
    OscWiFi.update();
    getIMU();
    showIMU();
    sendIMU();

    // 画面タッチで index 切り替え
    if (digitalRead(TP_PIN_PIN) == HIGH) {
        if (!pressed) {
          pressed = true;
          tft.fillScreen(TFT_BLACK);
          vmt.index++;
          if (vmt.index >= 3){
            vmt.index = 0;
          }          
        }
    } else {
        pressed = false;
    }
    delay(10);
}