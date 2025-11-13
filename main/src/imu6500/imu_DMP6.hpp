
#include "pins.hpp"
#include <Arduino.h>
#include "driver_mpu6500.h"
namespace imu6500_dmp
{
  typedef struct MotionDtect
  {
    bool motionInterrupt = false;
    bool x = false;
    bool y = false;
    bool z = false;
    bool yaw = false;
    bool pitch = false;
    bool roll = false;
    uint32_t ts = 0;
    operator bool() { return motionInterrupt || x || y || z || yaw || pitch || roll; }
    void reset()
    {
      motionInterrupt = false;
      x = false;
      y = false;
      z = false;
      roll = false;
      pitch = false;
      yaw = false;
      ts = 0;
    }
  } MotionDtect_t;

  enum imuSetupType
  {
    DMP,
    WOM,
    NA
  };

  typedef struct baseline_t
  {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    bool ready = false;
    uint64_t last_reset = 0;
  } baseline_t;

  bool LoadImuPreferences();

  bool imu_setup(imuSetupType st);
  MotionDtect_t getMotion();
  uint64_t getLastMovedTimestamp();
  void resetBaseline();
  baseline_t getbaseline();
  bool setupLowPowerMode();
  bool shutdown();
  bool restart(imuSetupType mode);
  bool SetWakeOnMotionThresh(float val);
  bool set_wom_lpf(mpu6500_accelerometer_low_pass_filter_t lp);
  bool set_wom_acc_output_rate(mpu6500_low_power_accel_output_rate_t rate);
  bool SetAccelCompare(bool val);
  // getters for current WOM configuration
  mpu6500_accelerometer_low_pass_filter_t get_wom_lpf();
  mpu6500_low_power_accel_output_rate_t get_wom_acc_output_rate();
  // save current WOM related settings to preferences (WOM_THR, WOM_LPF, WOM_RATE)
}