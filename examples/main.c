#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

int main() {
  stdio_init_all();

  printf("Setting up i2c...\n");
  // init i2c and the xshut gpio pin
  i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  //gpio_set_dir(3, GPIO_OUT);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  printf("i2c done\n");

  VL53L0X_Dev_t MyDevice;
  VL53L0X_Dev_t* pMyDevice = &MyDevice;

  pMyDevice->i2c = i2c_default;
  pMyDevice->address = 0x29;

  VL53L0X_DataInit(&MyDevice); // Data initialization

  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
  int i;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;

  printf ("Call of VL53L0X_StaticInit\n");
  Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
  printf ("status : %d\n", Status);

  printf ("Call of VL53L0X_PerformRefCalibration\n");
  Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal); // Device Initialization
  printf ("status : %d\n", Status);

  printf ("Call of VL53L0X_PerformRefSpadManagement\n");
  Status = VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads); // Device Initialization
  printf ("status : %d\n", Status);
  printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);

  // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
  printf ("Call of VL53L0X_SetDeviceMode\n");
  Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  printf ("status : %d\n", Status);

  Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
  printf ("status : %d\n", Status);

  Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  printf ("status : %d\n", Status);

  Status = VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,	(FixPoint1616_t)(0.1*65536));
  printf ("status : %d\n", Status);

  Status = VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,	(FixPoint1616_t)(60*65536));
  printf ("status : %d\n", Status);

  Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 33000);
  printf ("status : %d\n", Status);

  Status = VL53L0X_SetVcselPulsePeriod(pMyDevice, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  printf ("status : %d\n", Status);

  Status = VL53L0X_SetVcselPulsePeriod(pMyDevice, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  printf ("status : %d\n", Status);

  printf ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
  /*
  for(i=0;i<5;i++){
    VL53L0X_PerformSingleRangingMeasurement(pMyDevice, &RangingMeasurementData);
   printf("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);
  }
  */
  ///*
  while (1) {
    Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice, &RangingMeasurementData);
    printf ("status : %d", Status);
    printf("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);
  }
  //*/
}
