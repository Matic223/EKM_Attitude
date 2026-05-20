
#include <Arduino.h>
#include <ICM42670P.h>
#include "../src/EKM_att.h"



//nastavitev timerja
hw_timer_t* timer0 = NULL;
hw_timer_t* timer1 = NULL;

//test dolzine interupta za ICM
unsigned long razlika_test;

//primerjava baro klm
float Surov_bar;
float Baro_dt;

//namesto delay bo millies za izpis na serial plotter v main
unsigned long prejsniMIllis=0;
unsigned long mill;
unsigned long milis_dt=100; //ms;

//ICM
ICM42670 IMU(Wire,0);
 inv_imu_sensor_event_t imu_event;
 inv_imu_sensor_event_t imu_event_Baro;
 //spremenljivka za kalibracijo
 inv_imu_sensor_event_t imu_event_calib;
//BARO
#define SEALEVELPRESSURE_HPA (1013.25)  //1013.25

//NASTAVITEV TIMERJA
float dt_test=0.001;
hw_timer_t * timer = NULL;
volatile bool imu_update_requested_RollPitch = false;
volatile bool imu_update_requested_Alt = false;

////////////////////////31.3.2026//////////////////////////////////
// nastavitev variable oz. objekta za KLM output
kalmanICMData KLM_data_output;
double small_bias = 2;

//GyroBias
GyroBias Bias_data;

//Baro spremenljivka oz objekt za struct


///////////////////////////
//timer interrupt za roll pitch kalman
void IRAM_ATTR onTimer0(){
   imu_update_requested_RollPitch = true;  // signal loop() to do the work
 
}

//timer interrupt za roll pitch kalman
void IRAM_ATTR onTimer1(){
   imu_update_requested_Alt = true;  // signal loop() to do the work
 
}

void setup() {

  /////////////////////////////INICIALIZACIJA BAROMETRA/////////////////////////////////////////
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

 
  /////////////////////////////////INICIALIZACIJA ICM/////////////////////////////////////////////////
  int ret;
  // Initializing the ICM42670
  
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM42670 initialization failed: ");
    Serial.println(ret);
    while(1);
  }
  Serial.print("ICM42670 initialization succesful \n");
  // Accel ODR = 100 Hz and Full Scale Range = 16G
  IMU.startAccel(100,16);
  // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
  IMU.startGyro(100,2000); 
  // oba sta v Low Noise mode
  // Wait IMU to start
  
  //////////////////////////////////// ININCIALIZACIJA TIMERJA 0 //////////////////////////////
Serial.println("start timer 0");
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered 
  timerAlarmWrite(timer0, 10000, true); // 10000 * 1 us = 10 Ms=100Hz, autoreload true
  timerAlarmEnable(timer0); // enable

   //////////////////////////////////// ININCIALIZACIJA TIMERJA 1 //////////////////////////////
Serial.println("start timer 1");
  timer1 = timerBegin(1, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTimer1, true); // edge (not level) triggered 
  timerAlarmWrite(timer1, 20000, true); // 20000 * 1 us = 20 Ms=50Hz, autoreload true
  timerAlarmEnable(timer1); // enable

  delay(100);
  
///////// preracun biasa traja 100*0,01sec= 1 sekunda mirovanja za kalibracijo//////////////
for(int i=0;i < 100;i++){

 IMU.getDataFromRegisters(imu_event_calib);
 calibrateGyroBias(100,imu_event_calib.gyro[0],imu_event_calib.gyro[1],imu_event_calib.gyro[2], &Bias_data);
  delay(10);//100hz
}
Serial.print("biasx:");
Serial.print(Bias_data.gyroBiasX);
Serial.print("biasy:");
Serial.print(Bias_data.gyroBiasY);
Serial.print("biasz:");
Serial.print(Bias_data.gyroBiasZ);

//delay (100);  //za vizualizacijo v serial terminal
///////////////////////////kompas init/////////////////////////////////////////
Wire.begin();
resetEKF();
/* 
  Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);

  Serial.println("CALIBRATING. Keep moving your sensor...");
  compass.calibrate();

  Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
  Serial.println();
  Serial.print("compass.setCalibrationOffsets(");
  Serial.print(compass.getCalibrationOffset(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(2));
  Serial.println(");");
  Serial.print("compass.setCalibrationScales(");
  Serial.print(compass.getCalibrationScale(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(2));
  Serial.println(");");
  */
}

void loop() {
//////////////////////////////interrupt rutina v main za pitch roll//////////////////////////
  if(imu_update_requested_RollPitch){
    //unsigned long test= millis();
    //unsigned long prej_test;

    imu_update_requested_RollPitch= false;
    IMU.getDataFromRegisters(imu_event);
  
static unsigned long last_us = 0;  
float dt = 0.01f;  //za prvi run
unsigned long now_us = micros(); // Reads the current time (in microseconds) since the Arduino started running.

if (last_us > 0) {
  dt = (now_us - last_us) / 1e6f;  //iz microsecudn v sekunde
}
last_us = now_us;
kalmanAngle((float)imu_event.accel[0], 
  -(float)imu_event.accel[1],
  (float)imu_event.accel[2],
  -(float)imu_event.gyro[0],
  (float)imu_event.gyro[1],
  (float)imu_event.gyro[2],
        dt,
&KLM_data_output, Bias_data);


//kompas branje za tilt comoensated heading

     // Z stays the same
  //azimuth = compass.getAzimuth(); // degrees 0–360
  //bearing= compass.getBearing(azimuth);

  // roll = x[0], pitch = x[1] from your Kalman filter (in degrees)
   //float roll  = x[0] * DEG_TO_RAD;
   //float pitch = (x[1] * DEG_TO_RAD);
    float roll  = KLM_data_output.roll * DEG_TO_RAD;
    float pitch = (KLM_data_output.pitch * DEG_TO_RAD);


//azimuth=-azimuth;
  }

  //////////////////////////////////interrupt rutina za altitude//////////////////////
if(imu_update_requested_Alt){
    unsigned long test= millis();
    unsigned long prej_test;

    imu_update_requested_Alt= false;
    IMU.getDataFromRegisters(imu_event_Baro);
  

static unsigned long last_us = 0;
float dt = 0.02f;
unsigned long now_us = micros();

if (last_us > 0) {
  dt = (now_us - last_us) / 1e6f;
}
last_us = now_us;
/////////////////////inic baro//////////////////////////

//za preverjanje vrednsoti v serial
Baro_dt=dt;
  }


  mill=millis();

  if(mill-prejsniMIllis>=milis_dt){
    prejsniMIllis=mill;
 
    //9.1.2026
    int a;
  
  // Read compass values
  
a=a+360;
  }
  Serial.print(">");

   // Output
   
  Serial.print("Roll:");
  Serial.print(KLM_data_output.roll);
  Serial.print(",");
//
  Serial.print("Pitch:");
  Serial.print(KLM_data_output.pitch);
  Serial.print(",");
  
//
  //Serial.print("rateX:");
  //Serial.print((float)imu_event.gyro[0]/ 16.4f);
  //Serial.print(",");
//
  //Serial.print("rateY:");
  //Serial.print(-(float)imu_event.gyro[1]/ 16.4f);
  //Serial.print(",");
  /*
  Serial.print("Rolldeg:");
  Serial.print(ThetaRoll);

  Serial.print(",");
  Serial.print("pitchdeg:");
  Serial.print(ThetaPitch);
Serial.print(",");

  Serial.print("visbar:");
  Serial.print(x1[0]);
  
  Serial.print(",");
   Serial.print("speedishow:");
  Serial.print(x1[1]);
  

Serial.print(",");
  Serial.print("vis_Rare:");
  Serial.print(Surov_bar);
*/
  //Serial.print(",");
  Serial.print("accx:");
  Serial.print((float)imu_event.accel[0]); //BaroOut.KalmanAlt

  Serial.print(",");
  Serial.print("accy:");
  Serial.print((float)imu_event.accel[1]);  //BaroOut.KalmanspeedZ

  Serial.print(",");
  Serial.print("accz:");
  Serial.print((float)imu_event.accel[2]);  //BaroOut.KalmanspeedZ
  
  Serial.print(",");
  Serial.print("gyrox:");
  Serial.print(-(float)imu_event.gyro[0]); //BaroOut.KalmanAlt

  Serial.print(",");
  Serial.print("gyroy:");
  Serial.print((float)imu_event.gyro[1]);  //BaroOut.KalmanspeedZ

  Serial.print(",");
  Serial.print("gyroz:");
  Serial.print(-(float)imu_event.gyro[2]);  //BaroOut.KalmanspeedZ
  /*
  Serial.print(",");
  Serial.print("BiasX:");
  Serial.print(gyroBiasX);
  Serial.print(",");
  Serial.print("BiasY:");
  Serial.print(gyroBiasY);
  Serial.print(",");
  Serial.print("BiasZ:");
  Serial.print(gyroBiasZ);
  Serial.print(",");
  /* 
 Serial.print("Raw Mx="); Serial.print(compass.getX());
Serial.print(" My="); Serial.print(compass.getY());
Serial.print(" Mz="); Serial.println(compass.getZ());
*/
//Serial.print("Roll="); Serial.print(x[0]);
//Serial.print(" Pitch="); Serial.print(-x[1]);
//Serial.print(" Azimuth="); Serial.println(azimuth);
 /*
Serial.print("azimuth:");
  Serial.print(a);
  Serial.print(",");
  Serial.print("mx:");
  Serial.print(Mx);
  Serial.print(",");
  Serial.print("my:");
  Serial.print(My);
  Serial.print(",");
  Serial.print("mz:");
  Serial.print(Mz);
  //Serial.print("bearing:");
  //Serial.println(bearing);
  */
  Serial.println(); // Writes \r\n
  //Serial.print(",");

 /////////////////////////////////////////////////dodaj zvezdica slash

/*
  Serial.print("visina_real:");
  Serial.print(x1[0]);
Serial.print(",");
  Serial.print("rate_real:");
  Serial.print(x1[1]);

  Serial.print(",");
  Serial.print("Rolldeg:");
  Serial.print(ThetaRoll);

  Serial.print(",");
  Serial.print("pitchdeg:");
  Serial.print(ThetaPitch);
  Serial.print(",");
  Serial.print("dolzina_suba:");
  Serial.print(razlika_test);
 Serial.println(); // Writes \r\n

  // Run @ ODR 100Hz
  //delay(50);
  */
prejsniMIllis=mill;
  }



////////////////////////////////
/*
 *
 * Copyright (c) [2022] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *

#include <Arduino.h>
#include <ICM42670P.h>

extern "C" {
  #include "esp_intr_alloc.h"
  #include "soc/gpio_reg.h"
  #include "driver/periph_ctrl.h"
  #include "soc/soc.h"
  #include "soc/gpio_struct.h"
  #include "driver/gpio.h"
}


#define IMU_ADDR 0x68        // ICM42670 default address
#define IMU_INT_PIN 39       // Interrupt pin

volatile bool imu_interrupt_triggered = false;

// ISR
void IRAM_ATTR imu_gpio_isr(void* arg) {
  imu_interrupt_triggered = true;
}

// Low-level I²C register read
uint8_t readIMURegister(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // repeated start
  Wire.requestFrom(IMU_ADDR, 1);
  return Wire.read();
}

// Low-level I²C register write
void writeIMURegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();

  // Configure IMU INT1: active HIGH, push-pull
  writeIMURegister(0x14, 0x01);  // INTF_CONFIG: INT1 = push-pull, active HIGH
  writeIMURegister(0x15, 0x08);  // INT_SOURCE0: enable DATA_RDY interrupt on INT1
  writeIMURegister(0x16, 0x00);  // INT_CONFIG: push-pull

  // Print register values for verification
  uint8_t r1 = readIMURegister(0x14);
  uint8_t r2 = readIMURegister(0x15);
  uint8_t r3 = readIMURegister(0x16);
  Serial.printf("INTF_CONFIG (0x14): 0x%02X\n", r1);
  Serial.printf("INT_SOURCE0 (0x15): 0x%02X\n", r2);
  Serial.printf("INT_CONFIG (0x16): 0x%02X\n", r3);

  // Enable accel & gyro for data ready signal to be meaningful
  writeIMURegister(0x1F, 0x03);  // PWR_MGMT0: accel & gyro to Low Noise mode
  delay(100); // Wait sensor startup

  // Set ODR for gyro and accel
  writeIMURegister(0x20, 0x05);  // GYRO_CONFIG0: 100 Hz, 2000 dps
  writeIMURegister(0x21, 0x05);  // ACCEL_CONFIG0: 100 Hz, 16g

  // GPIO39 interrupt config (rising edge)
  pinMode(IMU_INT_PIN, INPUT);
  gpio_set_intr_type((gpio_num_t)IMU_INT_PIN, GPIO_INTR_POSEDGE);
  gpio_install_isr_service(0); // 0 = no flags
  gpio_isr_handler_add((gpio_num_t)IMU_INT_PIN, imu_gpio_isr, NULL);

  Serial.println("ICM42670 configured, waiting for interrupts on GPIO39...");
}

void loop() {
  if (imu_interrupt_triggered) {
    imu_interrupt_triggered = false;

    // Optional: Read INT_STATUS to confirm source
    uint8_t status = readIMURegister(0x2D);
    Serial.printf("Interrupt! INT_STATUS: 0x%02X\n", status);
  }
}
*/
