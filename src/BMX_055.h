#ifndef BMX_055_H
#define BMX_055_H

#include <stdint.h>
#include "esp_err.h"

struct bmx055_mag_trim_registers {
    int8_t dig_x1;
    int8_t dig_y1;
    int8_t dig_x2;
    int8_t dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2;
    int16_t dig_z3;
    int16_t dig_z4;
    uint8_t dig_xy1;
    int8_t dig_xy2;
    uint16_t dig_xyz1;
};

esp_err_t i2c_master_init(void);
void bmx055_init(void);
esp_err_t BMX055_ReadAccelRaw(int16_t *x, int16_t *y, int16_t *z);
esp_err_t BMX055_ReadGyroRaw(int16_t *x, int16_t *y, int16_t *z);
esp_err_t BMX055_ReadMagRaw(int16_t *x, int16_t *y, int16_t *z);
esp_err_t BMX055_ReadMagCompensated(float *x, float *y, float *z);

void bmx055_accel_read_task(void *pvParameters);
void bmx055_gyro_read_task(void *pvParameters);
void bmx055_mag_read_task(void *pvParameters);

#endif /* BMX_055_H */