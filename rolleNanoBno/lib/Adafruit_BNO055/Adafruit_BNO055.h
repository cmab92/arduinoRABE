// cb, 01.03.18
// based on official Adafruit_BNO055.h

#ifndef __Adafruit_BNO055_H__
#define __Adafruit_BNO055_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"

#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

// BNO constants (modes etc.)
#define ADDRESS                      0x29

#define ID                           0xa0
#define CONFIG_OPR                   0x00 // on OPR_MODE_REG
#define NDOF_FMC_OFF_OPR             0x0b // on OPR_MODE_REG
#define NDOF_OPR                     0x0c // on OPR_MODE_REG
#define IMU_MODE                     0x08 // on OPR_MODE_REG
#define RESET                        0x20 // on SYS_TRIGGER_REG
#define NORMAL_PWR                   0x00 // on PWR_MODE_REG
#define EXTAL                        0x80 // on SYS_TRIGGER_REG
#define UNIT_SEL_ADDR                0x3b

// System registers
#define STATUS_REG                   0x00
#define OPR_MODE_REG                 0x3d
#define SYS_TRIGGER_REG              0x3f
#define PWR_MODE_REG                 0x3e
#define PAGE_ID_REG                  0x07

// Accelerometer Offset registers
#define ACCEL_OFFSET_X_LSB_ADDR      0x55
#define ACCEL_OFFSET_X_MSB_ADDR      0x56
#define ACCEL_OFFSET_Y_LSB_ADDR      0x57
#define ACCEL_OFFSET_Y_MSB_ADDR      0x58
#define ACCEL_OFFSET_Z_LSB_ADDR      0x598
#define ACCEL_OFFSET_Z_MSB_ADDR      0x5a

// Magnetometer Offset registers
#define MAG_OFFSET_X_LSB_ADDR        0x5b
#define MAG_OFFSET_X_MSB_ADDR        0x5c
#define MAG_OFFSET_Y_LSB_ADDR        0x5d
#define MAG_OFFSET_Y_MSB_ADDR        0x5e
#define MAG_OFFSET_Z_LSB_ADDR        0x5f
#define MAG_OFFSET_Z_MSB_ADDR        0x60

// Gyroscope Offset registers
#define GYRO_OFFSET_X_LSB_ADDR       0x61
#define GYRO_OFFSET_X_MSB_ADDR       0x62
#define GYRO_OFFSET_Y_LSB_ADDR       0x63
#define GYRO_OFFSET_Y_MSB_ADDR       0x64
#define GYRO_OFFSET_Z_LSB_ADDR       0x65
#define GYRO_OFFSET_Z_MSB_ADDR       0x66

// Radius registers
#define ACCEL_RADIUS_LSB_ADDR        0x67
#define ACCEL_RADIUS_MSB_ADDR        0x68
#define MAG_RADIUS_LSB_ADDR          0x69
#define MAG_RADIUS_MSB_ADDR          0x6a

// Interrupt registers
#define ACC_HG_THRES_ADDR            0x14
#define ACC_HG_DURATION_ADDR         0x13
#define ACC_INT_SETTINGS_ADDR        0x12
#define INT_EN_ADDR                  0x10
#define INT_MSK_ADDR                 0x0f

// accel data register
#define ACCEL_DATA_X_LSB_ADDR        0x08
#define ACCEL_DATA_X_MSB_ADDR        0x09
#define ACCEL_DATA_Y_LSB_ADDR        0x0a
#define ACCEL_DATA_Y_MSB_ADDR        0x0b
#define ACCEL_DATA_Z_LSB_ADDR        0x0c
#define ACCEL_DATA_Z_MSB_ADDR        0x0d

// Mag data register
#define MAG_DATA_X_LSB_ADDR          0x0e
#define MAG_DATA_X_MSB_ADDR          0x0f
#define MAG_DATA_Y_LSB_ADDR          0x10
#define MAG_DATA_Y_MSB_ADDR          0x11
#define MAG_DATA_Z_LSB_ADDR          0x12
#define MAG_DATA_Z_MSB_ADDR          0x13

// Gyro data registers
#define GYRO_DATA_X_LSB_ADDR         0x14
#define GYRO_DATA_X_MSB_ADDR         0x15
#define GYRO_DATA_Y_LSB_ADDR         0x16
#define GYRO_DATA_Y_MSB_ADDR         0x17
#define GYRO_DATA_Z_LSB_ADDR         0x18
#define GYRO_DATA_Z_MSB_ADDR         0x19

// Euler data registers
#define EULER_H_LSB_ADDR             0x1a
#define EULER_H_MSB_ADDR             0x1b
#define EULER_R_LSB_ADDR             0x1c
#define EULER_R_MSB_ADDR             0x1d
#define EULER_P_LSB_ADDR             0x1e
#define EULER_P_MSB_ADDR             0x1f

/* Quaternion data registers */
#define QUATERNION_DATA_W_LSB_ADDR   0x20
#define QUATERNION_DATA_W_MSB_ADDR   0x21
#define QUATERNION_DATA_X_LSB_ADDR   0x22
#define QUATERNION_DATA_X_MSB_ADDR   0x23
#define QUATERNION_DATA_Y_LSB_ADDR   0x24
#define QUATERNION_DATA_Y_MSB_ADDR   0x25
#define QUATERNION_DATA_Z_LSB_ADDR   0x26
#define QUATERNION_DATA_Z_MSB_ADDR   0x27

// Linear acceleration data registers
#define LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define LINEAR_ACCEL_DATA_X_MSB_ADDR 0x29
#define LINEAR_ACCEL_DATA_Y_LSB_ADDR 0x2A
#define LINEAR_ACCEL_DATA_Y_MSB_ADDR 0x2B
#define LINEAR_ACCEL_DATA_Z_LSB_ADDR 0x2C
#define LINEAR_ACCEL_DATA_Z_MSB_ADDR 0x2D

// Gravity data register
#define GRAVITY_DATA_X_LSB_ADDR      0x2E
#define GRAVITY_DATA_X_MSB_ADDR      0x2F
#define GRAVITY_DATA_Y_LSB_ADDR      0x30
#define GRAVITY_DATA_Y_MSB_ADDR      0x31
#define GRAVITY_DATA_Z_LSB_ADDR      0x32
#define GRAVITY_DATA_Z_MSB_ADDR      0x33

class Adafruit_BNO055 : public Adafruit_Sensor
{
  public:

    typedef enum // adafruit_vector_type_t
    {
      VECTOR_ACCELEROMETER = ACCEL_DATA_X_LSB_ADDR,
      VECTOR_MAGNETOMETER  = MAG_DATA_X_LSB_ADDR,
      VECTOR_GYROSCOPE     = GYRO_DATA_X_LSB_ADDR,
      VECTOR_EULER         = EULER_H_LSB_ADDR,
      VECTOR_LINEARACCEL   = LINEAR_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_GRAVITY       = GRAVITY_DATA_X_LSB_ADDR
    } adafruit_vector_type_t;

    typedef enum // adafruit_bno055_reg_t   page 0
    {
      /* Page id register definition */
      BNO055_PAGE_ID_ADDR                                     = 0X07,

      /* PAGE0 REGISTER DEFINITION START*/
      BNO055_CHIP_ID_ADDR                                     = 0x00,
      BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
      BNO055_MAG_REV_ID_ADDR                                  = 0x02,
      BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
      BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
      BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
      BNO055_BL_REV_ID_ADDR                                   = 0X06,

      /* Accel data register */
      BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
      BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
      BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
      BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
      BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
      BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,

      /* Mag data register */
      BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
      BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
      BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
      BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
      BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
      BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,

      /* Gyro data registers */
      BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
      BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
      BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
      BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
      BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
      BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,

      /* Euler data registers */
      BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
      BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
      BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
      BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
      BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
      BNO055_EULER_P_MSB_ADDR                                 = 0X1F,

      /* Quaternion data registers */
      BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
      BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
      BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
      BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
      BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
      BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
      BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
      BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,

      /* Linear acceleration data registers */
      BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
      BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
      BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
      BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
      BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
      BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,

      /* Gravity data registers */
      BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
      BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
      BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
      BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
      BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
      BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

      /* Temperature data register */
      BNO055_TEMP_ADDR                                        = 0X34,

      /* Status registers */
      BNO055_CALIB_STAT_ADDR                                  = 0X35,
      BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
      BNO055_INTR_STAT_ADDR                                   = 0X37,

      BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
      BNO055_SYS_STAT_ADDR                                    = 0X39,
      BNO055_SYS_ERR_ADDR                                     = 0X3A,

      /* Unit selection register */
      BNO055_UNIT_SEL_ADDR                                    = 0X3B,
      BNO055_DATA_SELECT_ADDR                                 = 0X3C,

      /* Mode registers */
      BNO055_OPR_MODE_ADDR                                    = 0X3D,
      BNO055_PWR_MODE_ADDR                                    = 0X3E,

      BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
      BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

      /* Axis remap registers */
      BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
      BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

      /* SIC registers */
      BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
      BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
      BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
      BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
      BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
      BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
      BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
      BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
      BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
      BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
      BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
      BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
      BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
      BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
      BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
      BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
      BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
      BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54

    } adafruit_bno055_reg_t;

    Adafruit_BNO055 ( int32_t sensorID = -1, uint8_t address = (ADDRESS) );

    bool            writeBNO       ( bool page, byte reg, byte value );
    byte            readBNO        ( bool page, byte reg );

    imu::Vector<3>  getVector ( adafruit_vector_type_t vector_type );
    imu::Quaternion getQuat   ( void );

    bool  getEvent  ( sensors_event_t* );
    void  getSensor ( sensor_t* );
  private:

    uint8_t _address;
    int32_t _sensorID;
    byte  read8     ( byte reg );
    bool  write8    ( byte reg, byte value );
    bool  readLen   ( byte reg, byte* buffer, uint8_t len );

};

#endif
