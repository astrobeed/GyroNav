/*
  GyroNav https://github.com/astrobeed/GyroNav

  Sensore assoluto assetto e orientamento
  per robot mobili e sistemi di navigazione

  Misura Pitch e Roll nel range +/- 180°
  Misura orientamento rispetto al Nord magnetico 0-360°
  Precisione Pitch e Roll +/- 0.1°, stabilità +/- 0.1°
  Precione orientamento +/- 0.5°, stabilità +/- 0.5°
  Il valore di Pitch e Roll viene fornito in gradi con un decimale,
  risoluzione 0.1°.
  Il valore dell'orientamento viene fornito arrotondato al grado,
  risoluzione 1°
  La compensazione dell'orientamento è valida con valori di Pitch e Roll
  compresi tra +/- 45°, rispetto al piano in bolla, con errore massimo 
  compreso tra +/- 2.5° sul valore dell'orientamento.

  Sensori utilizzati :
  MPU650 + HMC5883L
   oppure
  MPU9150

  Changelog:
    04-11-2015 - alfa release
    10-05-2016 - beta 1
    18-06-2016 - beta 2
    12-08-2016 - rc 1
    25-02-2017 - v 1.0, fist stable release
    27-02-2017 - v 1.1, aggiunta media mobile magnetometro.

  Librerie indispensabili:
  I2Cdev
  MPU6050
  HMC5883L
  MPU9150

  Le librerie MPUxxx e HMCxxxx fanno parte della I2Cdev.
  Assicurarsi di aver installato la relese più recente delle librerie,
  l'uso di versioni non aggiornate, sopratutto per gli MPUxxxx, può
  causare errori nei dati acquisiti e instabilità degli stessi.

  Repository ufficiale I2Cdev https://github.com/jrowberg/i2cdevlib

  Compilato e testato con IDE 8.x
  La compilazione con IDE precendenti alla 8.0
  può causare errori di compilazione.
  Se necessario aggiornare l'IDE.

  ToDo:
  Gui per calibrazione sensori.
  Sensor fusion tra Yaw della IMU e heading magnetometro.
  Completare gestione MPU9150.
*/

/* ============================================

  GyroNav code is placed under the MIT license
  Copyright (c) 2017 MdA Project aka Astrobeed

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// configurazione sensori e modo operativo
#include "config.h"

// Arduino Wire library is required if I2Cdev
// used in I2Cdev.h
#include "Wire.h"
// I2Cdev library
#include "I2Cdev.h"

// usa MPU6050 + HCM5883L
#ifdef IMU_6050
// MPU6050 library
#include "MPU6050_6Axis_MotionApps20.h"
// HMC5883L Compass Library
#include <HMC5883L.h>
#endif

// usa MPU9150
#ifdef IMU_9150
#include "MPU9150_9Axis_MotionApps41.h"
#endif

// AD0 low  = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69 (Drotek board)
MPU6050 mpu(0x69); // <-- use for AD0 high
// MPU6050 mpu;    // <-- use for AD0 low

#define LED_PIN 13
bool blinkState = false;
// contatore per lampeggio led
byte led_freq;

// HMC5883L direzione in gradi
int heading_deg;
// magnetometro dati raw
int16_t mx, my, mz;
// buffer media mobile
int mag_media[16];
byte idx_mag;

// MPU control/status vars
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];
int16_t ax, ay, az;
int16_t gx, gy, gz;

//  costanti per la calibrazione giroscopio e accelerometro
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

// fattore di scala giroscopio
float    GYRO_FACTOR;
// fattore di scala accelerometro
float    ACCEL_FACTOR;

// millis buffer
unsigned long t_now = millis();

// setup
void setup() {
  // init I2C bus
  Wire.begin();
  // init UART
  Serial.begin(115200);

  // selezione del tipo di sensori usati
#ifdef IMU_6050
  // MPU6050 init
  MPU6050_init();
  // HMC5833L init
  MHC_init();
#endif

#ifdef IMU_9150
  // MPU9150_init();
  MPU9150_init;
#endif

  // calibrazione sensori
  calibrate_sensors();

  // LED pin output
  pinMode(LED_PIN, OUTPUT);
}

// main loop
void loop() {
  // blocco esecuzione in caso di errore IMU
  if (!dmpReady) return;

  // legge dati da MPU se ricevuto interrupt
  // sample time ~10 ms
  if (mpuInterrupt)
  {
    // dati raw accel/gyro dal MPU6050/MPU9150
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // estrae dati DMP
    MPU6050_Elaborate();
  }

  // lettura magnetometro
  // sample time 20 ms
  if (t_now < millis())
  {
    t_now = millis() + 20;
    Compass_Data();     // legge orientamento
    led_freq++;
    if (led_freq > 5)   // cambia stato led ogni 100 ms
    {
      Heartbeat();      // led blink
      led_freq = 0;
    }

    // formato dati per GUI Processing, valori in gradi
#ifdef datmode_0
    Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
    Serial.print(" ");
    Serial.print(ypr[1]*RADIANS_TO_DEGREES, 2);
    Serial.print(" ");
    // Serial.println(ypr[0], 2);
    // gradi magnetometro
    Serial.println(heading_deg);
#endif

    // formato dati per GUI Processing, valori in radianti
#ifdef datmode_1
    Serial.print(ypr[2], 1);
    Serial.print(" ");
    Serial.print(ypr[1], 1);
    Serial.print(" ");
    Serial.print(ypr[0], 1);
    Serial.print(" ");
    // Serial.println(ypr[0], 2);
    // gradi magnetometro
    Serial.println(heading_deg);
#endif

    // formato dati ascii
#ifdef datmode_2
    // IMU pitch
    Serial.print(ypr[1], 2);
    Serial.print(" : ");
    // IMU roll
    Serial.print(ypr[2], 2);
    Serial.print(" : ");
    // IMU jaw
    Serial.print(ypr[0], 2);
    Serial.print(" * ");
    // Magnetometro asse X raw
    Serial.print(mx);
    Serial.print(" : ");
    // Magnetometro asse Y raw
    Serial.print(my);
    Serial.print(" : ");
    // Magnetometro asse Z raw
    Serial.print(mz);
    Serial.print(" * ");
    // Magnetometro direzione del Nord magnetico in gradi
    Serial.println(heading_deg);
#endif

    // formato dati binario
#ifdef datmode_1
    // inserire formattazione dati
#endif
  }
}

// Led Heartbeat
void Heartbeat()
{
  // blink LED
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

/*
  CALIBRATION ROUTINE
  Simple calibration - just average first few readings
  to subtract from the later data
*/
void calibrate_sensors() {
  int       num_readings = 10;

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }

  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}

// MPU6050 INTERRUPT
void dmpDataReady() {
  mpuInterrupt = true;
}

