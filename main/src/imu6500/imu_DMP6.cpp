/*
  MPU6050 DMP6

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied,
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  The code includes an auto-calibration and offsets generator tasks. Different
  output formats available.

  This code is compatible with the teapot project by using the teapot output format.

  Circuit: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
  depends on the MPU6050's INT pin being connected to the Arduino's
  external interrupt #0 pin.

  The teapot processing example may be broken due FIFO structure change if using DMP
  6.12 firmware version.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki

*/

#include "imu6500/imu_DMP6.hpp"
#include "imu6500/driver_mpu6500_dmp.h"
#include "pins.hpp"
#include "wifi/wifi.hpp"
#include <atomic>
#include "power/power.hpp"
#include <Preferences.h>
#include "imu6500/motionCalc.hpp"

namespace imu6500_dmp
{

  /*---MPU6050 Control/Status Variables---*/
  uint64_t lastMoved_timestamp = 0;
  /*---Orientation/Motion Variables---*/
  imuSetupType ImuMode = NA;
  MotionDtect_t globalMotion = {};
  motion_t globalmotiondata = {};
  motion_t InterruptMotion = {};

  float MOTION_THRESHOLD_GX = 0.0075f; // sensitivity: ~0.03 g (~0.3 m/s^2)
  float MOTION_THRESHOLD_GY = 0.0075f; // sensitivity: ~0.03 g (~0.3 m/s^2)
  float MOTION_THRESHOLD_GZ = 0.0085f; // sensitivity: ~0.03 g (~0.3 m/s^2)

  float MOTION_THRESHOLD_ROLL = 0.5f;  // sensitivity: ~0.03 g (~0.3 m/s^2)
  float MOTION_THRESHOLD_YAW = 0.5f;   // sensitivity: ~0.03 g (~0.3 m/s^2)
  float MOTION_THRESHOLD_PITCH = 0.5f; // sensitivity: ~0.03 g (~0.3 m/s^2)
  float WOM_DET_THRESH = 15.0f;
  mpu6500_accelerometer_low_pass_filter_t WOM_LPF = MPU6500_ACCELEROMETER_LOW_PASS_FILTER_1;
  mpu6500_low_power_accel_output_rate_t WOM_RATE = MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_3P91;
  bool ACCEL_COMPARE = true;
  // Motion detection state
  // Baseline linear acceleration (gravity removed) in g's
  static baseline_t baseline;
  float dax = 0.0f, day = 0.0f, daz = 0.0f, droll = 0.0f, dyaw = 0.0f, dpitch = 0.0f;
  float qf[4];
  uint16_t calibrationDebounce = 0;
  uint16_t motionAfterBaselineCounter = 0;
  const uint16_t BASELINE_SAMPLES = 50;

  /*------Interrupt detection routine------*/
  std::atomic<bool> IMUInterrupt{false};
  std::atomic<bool> MPU_DMP_DATA_READY{false};
  std::atomic<bool> MPU_MTION_Interrupt{false};
  std::atomic<bool> imu_dmp_loop{false};
  SemaphoreHandle_t imuDataSemaphore, wireMutex, getterSem;
  void imu_DMP_loop(void *arg);

  void IRAM_ATTR IMUDataInterrupt()
  {
    IMUInterrupt = true;
  }

  void imu_Interrupt_loop(void *arg)
  {
    while (imu_dmp_loop)
    {
      if (IMUInterrupt)
      {
        auto res = xSemaphoreTake(wireMutex, pdMS_TO_TICKS(10));
        if (res == pdTRUE)
        {
          IMUInterrupt = false;
          mpu6500_dmp_irq_handler();
          xSemaphoreGive(wireMutex);
        }
      }
      delay(25);
    }
    vTaskDelete(NULL);
  }

  void MPU_InterruptCallback(uint8_t type)
  {
    switch (type)
    {
    case MPU6500_INTERRUPT_MOTION:
    {
      Serial.println("mpu6500: irq motion.");
      xSemaphoreTake(getterSem, pdMS_TO_TICKS(15));
      MPU_MTION_Interrupt = true;
      globalMotion.motionInterrupt = true;
      globalMotion.ts = millis();
      lastMoved_timestamp = millis();
      xSemaphoreGive(getterSem);
      break;
    }
    case MPU6500_INTERRUPT_DMP:
    {
      // Serial.println("mpu6500: irq DMP_READY");
      InterruptMotion.l = 5;
      if (mpu6500_dmp_read_motion(&InterruptMotion) == 0)
      {
        xSemaphoreTake(imuDataSemaphore, pdMS_TO_TICKS(10));
        globalmotiondata = InterruptMotion;
        xSemaphoreGive(imuDataSemaphore);
        MPU_DMP_DATA_READY = true;
        // Serial.printf("got data len=%d\n", motion.l);
      }
      break;
    }
    default:
      break;
    }
  }

  bool restart(imuSetupType mode)
  {
    bool ret = shutdown();
    ret = imu_setup(mode);
    return ret;
  }

  bool shutdown()
  {
    Serial.println("imu shutdown...");
    imu_dmp_loop = false;
    vTaskDelay(pdMS_TO_TICKS(30));
    xSemaphoreTake(wireMutex, pdMS_TO_TICKS(10));
    mpu6500_dmp_deinit();
    xSemaphoreGive(wireMutex);
    ImuMode = NA;
    return true;
  }
  void powerCycle()
  {
    auto &PMU = power::getPMU();
    PMU.disableDC5();
    delay(500);
    PMU.setDC5Voltage(3400);
    PMU.enableDC5();
  }

  bool imu_setup(imuSetupType mode)
  {
    if (imuDataSemaphore == NULL)
    {
      vSemaphoreCreateBinary(wireMutex);
      xSemaphoreGive(wireMutex);
      vSemaphoreCreateBinary(getterSem);
      xSemaphoreGive(getterSem);
      vSemaphoreCreateBinary(imuDataSemaphore);
      xSemaphoreGive(imuDataSemaphore);
    }
    LoadImuPreferences();
    uint8_t counter = 0;
    /*Verify connection*/
    ImuMode = mode;
    Serial.println(F("starting MPU6050 connection..."));
    if (ImuMode == WOM)
    {
      if (mpu6500_wom_init(MPU6500_INTERFACE_IIC,
                           MPU6500_ADDRESS_AD0_LOW,
                           MPU_InterruptCallback,
                           WOM_DET_THRESH,
                           WOM_LPF,
                           WOM_RATE,
                           ACCEL_COMPARE) != 0)
      {
        Serial.println("MPU6050 connection failed");
        // External row needle, 1400~3700mV // external supply from pmu to header
        powerCycle();
        if (mpu6500_wom_init(MPU6500_INTERFACE_IIC,
                             MPU6500_ADDRESS_AD0_LOW,
                             MPU_InterruptCallback,
                             WOM_DET_THRESH,
                             WOM_LPF,
                             WOM_RATE,
                             ACCEL_COMPARE) != 0)
        {
          Serial.println("MPU6050 connection failed again ");
          ImuMode = NA;
          return false;
        }
      }
      else
      {
        Serial.println("MPU6050 WOM successful");
      }
    }
    else
    {
      if (mpu6500_dmp_init(MPU6500_INTERFACE_IIC,
                           MPU6500_ADDRESS_AD0_LOW,
                           MPU_InterruptCallback, WOM_DET_THRESH, false) != 0)
      {
        Serial.println("MPU6050 connection failed");
        // External row needle, 1400~3700mV // external supply from pmu to header
        powerCycle();
        if (mpu6500_dmp_init(MPU6500_INTERFACE_IIC,
                             MPU6500_ADDRESS_AD0_LOW,
                             MPU_InterruptCallback, WOM_DET_THRESH, false) != 0)
        {
          Serial.println("MPU6050 connection failed again ");
          ImuMode = NA;
          return false;
        }
      }
      else
      {
        Serial.println("MPU6050 connection in DMP mode successful");
        xTaskCreate(imu_DMP_loop, "IMUloop", 8192, NULL, 1, NULL);
      }
    }
    detachInterrupt(MOTION_INTRRUPT_PIN);
    pinMode(MOTION_INTRRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(MOTION_INTRRUPT_PIN, IMUDataInterrupt, FALLING);
    imu_dmp_loop = true;
    xTaskCreate(imu_Interrupt_loop, "IMU", 8192, NULL, 1, NULL);

    return true;
  }

  uint64_t getLastMovedTimestamp()
  {
    xSemaphoreTake(getterSem, pdMS_TO_TICKS(30));
    uint64_t ts = lastMoved_timestamp;
    xSemaphoreGive(getterSem);
    return ts;
  }
  // Exposed function to report motion. Returns true once if motion detected since last call.
  MotionDtect_t getMotion()
  { // Atomically consume the flag
    xSemaphoreTake(getterSem, pdMS_TO_TICKS(30));
    MotionDtect_t str = globalMotion;
    globalMotion.reset();
    xSemaphoreGive(getterSem);
    return str;
  }

  bool LoadImuPreferences()
  {
    Preferences imuPref;
    imuPref.begin("imu", true);
    MOTION_THRESHOLD_GX = imuPref.getFloat("M_TH_GX", MOTION_THRESHOLD_GX);
    MOTION_THRESHOLD_GY = imuPref.getFloat("M_TH_GY", MOTION_THRESHOLD_GY);
    MOTION_THRESHOLD_GZ = imuPref.getFloat("M_TH_GZ", MOTION_THRESHOLD_GZ);
    MOTION_THRESHOLD_ROLL = imuPref.getFloat("M_TH_ROLL", MOTION_THRESHOLD_ROLL);
    MOTION_THRESHOLD_YAW = imuPref.getFloat("M_TH_YAW", MOTION_THRESHOLD_YAW);
    MOTION_THRESHOLD_PITCH = imuPref.getFloat("M_TH_PITCH", MOTION_THRESHOLD_PITCH);
    WOM_DET_THRESH = imuPref.getFloat("WOM_THR", WOM_DET_THRESH);
    ACCEL_COMPARE = imuPref.getBool("ACC_COMR", ACCEL_COMPARE);
    WOM_LPF = (mpu6500_accelerometer_low_pass_filter_t)imuPref.getUInt("WOM_LPF", (uint32_t)WOM_LPF);
    WOM_RATE = (mpu6500_low_power_accel_output_rate_t)imuPref.getUInt("WOM_RATE", (uint32_t)WOM_RATE);
    Serial.printf("WOM_DET_THRESH=%.2f ,WOM_LPF=%d ,  WOM_RATE= %d , ACCEL_COMPARE=%d \n", WOM_DET_THRESH, WOM_LPF, WOM_RATE, ACCEL_COMPARE);
    imuPref.end();
    return true;
  }

  bool SetAccelCompare(bool val)
  {
    bool ret = true;
    Preferences imuPref;
    imuPref.begin("imu");
    imuPref.putBool("ACC_COMR", val);
    imuPref.end();
    ACCEL_COMPARE = val;
    return ret;
  }

  bool SetWakeOnMotionThresh(float val)
  {
    bool ret = true;
    Preferences imuPref;
    imuPref.begin("imu");
    imuPref.putFloat("WOM_THR", val);
    imuPref.end();
    WOM_DET_THRESH = val;
    return ret;
  }

  bool set_wom_lpf(mpu6500_accelerometer_low_pass_filter_t lp)
  {
    WOM_LPF = lp;
    Preferences imuPref;
    imuPref.begin("imu", false);
    imuPref.putUInt("WOM_LPF", (uint32_t)WOM_LPF);
    imuPref.end();
    return true;
  }

  bool set_wom_acc_output_rate(mpu6500_low_power_accel_output_rate_t rate)
  {
    WOM_RATE = rate;
    Preferences imuPref;
    imuPref.begin("imu", false);
    imuPref.putUInt("WOM_RATE", (uint32_t)WOM_RATE);
    imuPref.end();
    return true;
  }

  mpu6500_accelerometer_low_pass_filter_t get_wom_lpf()
  {
    return WOM_LPF;
  }

  mpu6500_low_power_accel_output_rate_t get_wom_acc_output_rate()
  {
    return WOM_RATE;
  }
  bool GetAccelCompare()
  {
    return ACCEL_COMPARE;
  }


  baseline_t getbaseline()
  {
    return baseline;
  }

  void resetBaseline()
  {
    baseline.ready = false;
    motionAfterBaselineCounter = 0;
  }

  bool waitforBaseline()
  {
    if (!baseline.ready)
    {
      calibrationDebounce = 0;
      MPU_DMP_DATA_READY = false;
      MPU_MTION_Interrupt = false;
      float sx = 0.f, sy = 0.f, sz = 0.f, sroll = 0.0f, syaw = 0.0f, spitch = 0.0f;
      float syaw_sin = 0.0f, syaw_cos = 0.0f;
      float sroll_sin = 0.0f, sroll_cos = 0.0f;
      float spitch_sin = 0.0f, spitch_cos = 0.0f;
      uint16_t collected = 0;
      auto res = xSemaphoreTake(wireMutex, pdMS_TO_TICKS(10));
      if (res == pdTRUE)
      {
        mpu6500_dmp_resetFIFO();
        xSemaphoreGive(wireMutex);
      }
      Serial.printf("Baseline calibrate started \n\r");
      for (uint16_t i = 0; i < BASELINE_SAMPLES; ++i)
      {
        if (MPU_MTION_Interrupt)
        {
          Serial.printf("Baseline calibrate FAILED \n\r");
          return false;
        }
        motion_t data;
        while (!MPU_DMP_DATA_READY)
        {
          delay(30);
        };
        MPU_DMP_DATA_READY = false;
        xSemaphoreTake(imuDataSemaphore, pdMS_TO_TICKS(30));
        auto motionData = globalmotiondata;
        xSemaphoreGive(imuDataSemaphore);
        for (int i = 0; i < motionData.l; i++)
        {

          quat_q30_to_float(globalmotiondata.quat[i], qf);
          float a_world[3];
          rotate_accel_world(qf, globalmotiondata.accel_g[i], a_world);
          sx += fabs(a_world[0]);
          sy += fabs(a_world[1]);
          sz += fabs(a_world[2] - 1.0f);
          /*sx += motionData.accel_g[i][0];
            sy += motionData.accel_g[i][1];
            sz += motionData.accel_g[i][2];*/

          float yaw_rad = radians(motionData.yaw[i]);
          float roll_rad = radians(motionData.roll[i]);
          float pitch_rad = radians(motionData.pitch[i]);

          syaw_sin += sin(yaw_rad);
          syaw_cos += cos(yaw_rad);
          sroll_sin += sin(roll_rad);
          sroll_cos += cos(roll_rad);
          spitch_sin += sin(pitch_rad);
          spitch_cos += cos(pitch_rad);
          collected++;
        }
      }
      if (collected > 0)
      {
        baseline.ax = sx / collected;
        baseline.ay = sy / collected;
        baseline.az = sz / collected;
        baseline.yaw = degrees(atan2(syaw_sin / collected, syaw_cos / collected));
        baseline.pitch = degrees(atan2(spitch_sin / collected, spitch_cos / collected));
        baseline.roll = degrees(atan2(sroll_sin / collected, sroll_cos / collected));

        if (baseline.yaw < 0)
          baseline.yaw += 360.0f;
        if (baseline.pitch < 0)
          baseline.pitch += 360.0f;
        if (baseline.roll < 0)
          baseline.roll += 360.0f;
        baseline.ready = true;
        Serial.println("Motion detected baseline ready ");
        Serial.printf(" ax %.5f, ay %.5f, az %.5f, yaw %.5f, pitch %.5f, roll %.5f \n", baseline.ax, baseline.ay, baseline.az, baseline.yaw, baseline.pitch, baseline.roll);
        Serial.printf(" thx %.4f, thy %.4f, thz %.4f, yaw %.2f, pitch %.2f, roll %.2f \n",
                      MOTION_THRESHOLD_GX, MOTION_THRESHOLD_GY, MOTION_THRESHOLD_GZ, MOTION_THRESHOLD_YAW, MOTION_THRESHOLD_PITCH, MOTION_THRESHOLD_ROLL);
        baseline.last_reset = millis();
      }
    }

    return baseline.ready;
  }

  void imu_DMP_loop(void *arg)
  {
    lastMoved_timestamp = millis();
    baseline.last_reset = millis();
    Serial.println("Starting IMU DMP loop...");
    while (imu_dmp_loop)
    {
      if (((millis()) > (lastMoved_timestamp + 1500)) && ((millis()) > (baseline.last_reset + 15000)) && (motionAfterBaselineCounter > 30))
      {
        Serial.println("Periodic baseline calibration");
        Serial.flush();
        resetBaseline();
      }
      if (!waitforBaseline())
      {
        delay(500);
        continue;
      }
      /* Read a packet from FIFO */
      if (MPU_MTION_Interrupt)
      {
        MPU_MTION_Interrupt = false;
        Serial.println("interrupt mtion detected");
      }
      if (MPU_DMP_DATA_READY)
      { // Get the Latest packet
        MPU_DMP_DATA_READY = false;
        // // Compute delta from baseline
        xSemaphoreTake(imuDataSemaphore, pdMS_TO_TICKS(30));
        for (uint8_t i = 0; i < globalmotiondata.l; i++)
        {

          droll = angleDiff(globalmotiondata.roll[i], baseline.roll);
          dyaw = angleDiff(globalmotiondata.yaw[i], baseline.yaw);
          dpitch = angleDiff(globalmotiondata.pitch[i], baseline.pitch);
          quat_q30_to_float(globalmotiondata.quat[i], qf);
          float a_world[3];
          rotate_accel_world(qf, globalmotiondata.accel_g[i], a_world);
          float a_lin[3] = {fabs(a_world[0]), fabs(a_world[1]), fabs(a_world[2] - 1.0f)};
          // magnitude of linear accel (g)
          float mag = sqrtf((a_lin[0] * a_lin[0]) + (a_lin[1] * a_lin[1]) /*+ (a_lin[2] * a_lin[2])*/);

          /*dax = fabs(globalmotiondata.accel_g[i][0] - baseline.ax);
          day = fabs(globalmotiondata.accel_g[i][1] - baseline.ay);
          daz = fabs(globalmotiondata.accel_g[i][2] - baseline.az);*/

          dax = fabs(a_lin[0] - baseline.ax);
          day = fabs(a_lin[1] - baseline.ay);
          daz = fabs(a_lin[2] - baseline.az);
          xSemaphoreTake(getterSem, pdMS_TO_TICKS(30));
          if (dax > MOTION_THRESHOLD_GX)
          {
            globalMotion.x = true;
            Serial.printf("Motion  diff x= %.4f  \n", dax);
          }
          if (day > MOTION_THRESHOLD_GY)
          {
            globalMotion.y = true;
            Serial.printf("Motion diff y = %.4f \n", day);
          }
          if (daz > MOTION_THRESHOLD_GZ)
          {
            globalMotion.z = true;
            Serial.printf("Motion diff z = %.4f \n", daz);
          }
          if (droll > MOTION_THRESHOLD_ROLL)
          {

            globalMotion.roll = true;
            Serial.printf("Motion droll = %.4f \n", droll);
          }
          if (dyaw > MOTION_THRESHOLD_YAW)
          {

            globalMotion.yaw = true;
            Serial.printf("Motion dyaw = %.4f \n", dyaw);
          }
          if (dpitch > MOTION_THRESHOLD_PITCH)
          {
            globalMotion.pitch = true;
            Serial.printf("Motion dpitch = %.4f  \n", dpitch);
          }
          if (globalMotion)
          {
            lastMoved_timestamp = millis();
            if (droll > 1 || dpitch > 1 || dyaw > 1 || dax > 0.1f || day > 0.1f || daz > 0.1f)
            {
              if (calibrationDebounce++ > 15)
              {
                resetBaseline();
              }
            }
            motionAfterBaselineCounter++;
            // delay(50);
          }
          else
          {
            if (motionAfterBaselineCounter > 0)
              motionAfterBaselineCounter--;
          }
          xSemaphoreGive(getterSem);
        }
        xSemaphoreGive(imuDataSemaphore);
      }
      delay(50);
    }
    vTaskDelete(NULL);
  }
}
