# MS5837-PX4

MS5837 barometer driver for PX4 firmware. 

## To install:

1. In drv_sensor.h, add `#define DRV_BARO_DEVTYPE_MS5837		0x3F`. You may need to redefine `DRV_BARO_DEVTYPE_BMP280` as something else. 
   Or define `DRV_BARO_DEVTYPE_MS5837` as 0x63.

2. In CMakeLists.txt in folder src/drivers/barometer, add line `add_subdirectory(ms5837)`. 

3. In ROMFS/px4fmu_common/init.d/rc.sensors, add line `ms5837 start -a`. 

4. Confirm it works by going to NuttX Console and attempting to poll it for info, i.e. MS5837 info. 

## Note:

The driver is set at default to take 25 values and average them before it reports to uORB, as a very basic running average filter. 
To change how many values are collected before reporting, change `if (cnter == 25) in function `MS5837::collect()` to whatever value you'd like. 




