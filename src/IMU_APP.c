/*
 * File: imu_app.c
 * Brief:
 * IMU application layer ported from a generic MPU6050+compass backend
 * to the BMX055 driver (bmx055_iic.c/.h) on ESP32 / FreeRTOS.
 *
 * This file combines what used to be three source files (imu_app.c,
 * imu_filter.c, imu_math.c-as-header-inlines) into one: the Butterworth
 * filter implementation now lives directly below, since it exists only
 * to support this module's fusion pipeline.
 *
 * The update flow is unchanged from the original:
 * 1) read raw sensor data (now from BMX055, in raw counts),
 * 2) remove offsets,
 * 3) low-pass filter gyro/accel,
 * 4) run Madgwick AHRS,
 * 5) export Euler angles (pitch / roll / yaw).
 *
 * Function index:
 * 1. IMU_FilterSetCutoff()    - Compute Butterworth biquad coefficients.
 * 2. IMU_FilterApply()        - Apply one filter sample.
 * 3. IMU_App_CalibrateGyro()  - Estimate gyro zero bias while the board is still.
 * 4. IMU_App_ResetAttitude()  - Build the initial quaternion from accel + mag.
 * 5. IMU_App_UpdateDt()       - Refresh the loop delta time.
 * 6. IMU_App_UpdateSensors()  - Read and pre-process all sensor channels.
 * 7. IMU_App_MadgwickUpdate() - Run the AHRS fusion step.
 * 8. IMU_App_UpdateEuler()    - Convert quaternion to pitch/roll/yaw.
 * 9. IMU_App_Init()           - Initialize the whole IMU app.
 * 10. IMU_App_Update()        - Execute one update cycle.
 * 11. IMU_App_GetState()      - Return the latest state snapshot.
 * 12. IMU_App_GetPitch/Roll/Yaw() - Convenience scalar getters.
 * 13. IMU_App_StartTask()     - FreeRTOS task: update + periodic log.
 */

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "imu_app.h"
#include "bmx_055.h"

#define IMU_USE_MAGNETOMETER   1

/* Avoid relying on the non-standard M_PI macro (not guaranteed by C99
 * without platform-specific feature-test macros). */
#define IMU_FILTER_PI 3.14159265358979323846f

#define IMU_SAMPLE_HZ          200.0f
#define IMU_DEFAULT_DT         (1.0f / IMU_SAMPLE_HZ)

/* 1 / 16.384 LSB/deg/s for BMX055 initialized to +/- 2000 deg/s range */
#define GYRO_CALIBRATION_COFF  0.061035f

static const char *TAG = "IMU_APP";

static IMU_State s_imu;
static IMU_ButterParam s_accel_param;
static IMU_ButterParam s_gyro_param;
static IMU_ButterBuffer s_accel_buf[3];
static IMU_ButterBuffer s_gyro_buf[3];
static uint32_t s_last_update_us = 0;

static float s_beta = 0.45f;


/* ------------------------------------------------------------------------ */
/* 1-2. Butterworth low-pass filter (formerly imu_filter.c)                 */
/* ------------------------------------------------------------------------ */

void IMU_FilterSetCutoff(float sample_hz, float cutoff_hz, IMU_ButterParam *param)
{
    /* Standard bilinear-transform 2nd order Butterworth low-pass design. */
    float fr = sample_hz / cutoff_hz;
    float ohm = tanf(IMU_FILTER_PI / fr);
    float c = 1.0f + 2.0f * cosf(IMU_FILTER_PI / 4.0f) * ohm + ohm * ohm;

    param->b0 = (ohm * ohm) / c;
    param->b1 = 2.0f * param->b0;
    param->b2 = param->b0;
    param->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    param->a2 = (1.0f - 2.0f * cosf(IMU_FILTER_PI / 4.0f) * ohm + ohm * ohm) / c;
    param->a0 = 1.0f; /* unused directly, kept for completeness */
}

float IMU_FilterApply(float input, IMU_ButterBuffer *buf, const IMU_ButterParam *param)
{
    float output = param->b0 * input
                 + param->b1 * buf->x1
                 + param->b2 * buf->x2
                 - param->a1 * buf->y1
                 - param->a2 * buf->y2;

    buf->x2 = buf->x1;
    buf->x1 = input;
    buf->y2 = buf->y1;
    buf->y1 = output;

    return output;
}

/* ------------------------------------------------------------------------ */
/* Small local helpers replacing the platform layer                         */
/* ------------------------------------------------------------------------ */

static inline uint32_t IMU_Micros(void)
{
    return (uint32_t)esp_timer_get_time();
}

static inline void IMU_DelayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* Wrap the BMX055 raw reads into the IMU_Vector3f shape the rest of this
 * file expects, matching the call shape of the original MPU6050_Read(). */
static void IMU_ReadAccelGyro(IMU_Vector3f *gyro, IMU_Vector3f *accel)
{
    int16_t gx, gy, gz;
    int16_t ax, ay, az;

    if (BMX055_ReadGyroRaw(&gx, &gy, &gz) != ESP_OK) {
        ESP_LOGE(TAG, "Gyro read failed");
        gx = gy = gz = 0;
    }
    if (BMX055_ReadAccelRaw(&ax, &ay, &az) != ESP_OK) {
        ESP_LOGE(TAG, "Accel read failed");
        ax = ay = az = 0;
    }

    /* Math Frame: X = Phys Y, Y = -Phys X, Z = Phys Z
     * Swaps Pitch/Roll, maintains Right-Hand Rule, keeps +1g Z upward. */
    gyro->x = (float)gy;
    gyro->y = -(float)gx;
    gyro->z = (float)gz;

    accel->x = (float)ay;
    accel->y = -(float)ax;
    accel->z = (float)az;
}

static void IMU_ReadMag(IMU_Vector3f *mag)
{
    int16_t mx, my, mz;

    if (BMX055_ReadMagRaw(&mx, &my, &mz) != ESP_OK) {
        ESP_LOGE(TAG, "Mag read failed");
        mx = my = mz = 0;
    }

    /* BMX055 Mag alignment relative to Accel frame: X = A_X, Y = -A_Y, Z = -A_Z.
     * Applying Math Frame mapping (Math X = A_Y, Math Y = -A_X, Math Z = A_Z):
     * Math X = -my
     * Math Y = -mx
     * Math Z = -mz */
    mag->x = (float)mx;  
    mag->y = -(float)my; 
    mag->z = -(float)mz; 
}

/* ------------------------------------------------------------------------ */
/* 1. Gyro bias calibration                                                 */
/* ------------------------------------------------------------------------ */

static void IMU_App_CalibrateGyro(void)
{
    uint16_t i;
    IMU_Vector3f gyro_sum = {0};
    IMU_Vector3f gyro_raw;
    IMU_Vector3f accel_raw;

    /* Give sensors time to settle after power-up and I2C initialization. */
    IMU_DelayMs(200);
    for (i = 0; i < 200U; i++) {
        IMU_ReadAccelGyro(&gyro_raw, &accel_raw);
        /* Average many samples to estimate the static gyro bias. */
        gyro_sum.x += gyro_raw.x;
        gyro_sum.y += gyro_raw.y;
        gyro_sum.z += gyro_raw.z;
        IMU_DelayMs(5);
    }

    /* Save the startup bias so later updates can subtract it directly. */
    s_imu.gyro_offset.x = gyro_sum.x / 200.0f;
    s_imu.gyro_offset.y = gyro_sum.y / 200.0f;
    s_imu.gyro_offset.z = gyro_sum.z / 200.0f;
}

/* ------------------------------------------------------------------------ */
/* 2. Initial attitude from accel + mag                                     */
/* ------------------------------------------------------------------------ */

static void IMU_App_ResetAttitude(void)
{
    float roll_obs;
    float pitch_obs;
    float yaw_obs = 0.0f;
    float ax;
    float ay;
    float az;
    IMU_Vector3f mag;

    /* Use one fresh sensor snapshot to build the initial orientation estimate. */
    IMU_ReadAccelGyro(&s_imu.gyro_raw, &s_imu.accel_raw);
    IMU_ReadMag(&mag);
    s_imu.mag_raw = mag;

    ax = s_imu.accel_raw.x - s_imu.accel_offset.x;
    ay = s_imu.accel_raw.y - s_imu.accel_offset.y;
    az = s_imu.accel_raw.z - s_imu.accel_offset.z;

    /* Roll and pitch come from the gravity vector observed by the accelerometer. */
    roll_obs = -57.3f * atanf(ax * IMU_InvSqrt(ay * ay + az * az));
    pitch_obs = 57.3f * atanf(ay * IMU_InvSqrt(ax * ax + az * az));

#if IMU_USE_MAGNETOMETER
    float sin_pitch = sinf(pitch_obs * IMU_DEG2RAD);
    float cos_pitch = cosf(pitch_obs * IMU_DEG2RAD);
    float sin_roll = sinf(roll_obs * IMU_DEG2RAD);
    float cos_roll = cosf(roll_obs * IMU_DEG2RAD);
    float mxn;
    float myn;

    /* Tilt-compensate the magnetic vector before computing yaw. */
    mxn = mag.x * cos_roll + mag.z * sin_roll;
    myn = mag.x * sin_pitch * sin_roll
        + mag.y * cos_pitch
        - mag.z * sin_pitch * cos_roll;
    yaw_obs = atan2f(mxn, myn) * 57.296f;
#endif

    {
        float pitch = roll_obs * IMU_DEG2RAD;
        float roll = pitch_obs * IMU_DEG2RAD;
        float yaw = yaw_obs * IMU_DEG2RAD;

        /* Convert the initial Euler estimate into the quaternion state. */
        s_imu.quat.w = cosf(yaw * 0.5f) * cosf(pitch * 0.5f) * cosf(roll * 0.5f)
                     + sinf(yaw * 0.5f) * sinf(pitch * 0.5f) * sinf(roll * 0.5f);
        s_imu.quat.x = cosf(yaw * 0.5f) * cosf(pitch * 0.5f) * sinf(roll * 0.5f)
                     - sinf(yaw * 0.5f) * sinf(pitch * 0.5f) * cosf(roll * 0.5f);
        s_imu.quat.y = cosf(yaw * 0.5f) * sinf(pitch * 0.5f) * cosf(roll * 0.5f)
                     + sinf(yaw * 0.5f) * cosf(pitch * 0.5f) * sinf(roll * 0.5f);
        s_imu.quat.z = sinf(yaw * 0.5f) * cosf(pitch * 0.5f) * cosf(roll * 0.5f)
                     - cosf(yaw * 0.5f) * sinf(pitch * 0.5f) * sinf(roll * 0.5f);
    }
}

/* ------------------------------------------------------------------------ */
/* 3. Loop timing                                                           */
/* ------------------------------------------------------------------------ */

static void IMU_App_UpdateDt(void)
{
    uint32_t now_us = IMU_Micros();

    if (s_last_update_us == 0U) {
        s_imu.dt_s = IMU_DEFAULT_DT;
    } else {
        /* Trust the hardware timer exactly. Remove artificial bounds clamping
         * that multiplies integration errors during scheduler jitter. */
        s_imu.dt_s = (float)(now_us - s_last_update_us) / 1000000.0f;
        
        /* Only filter mathematically destructive states. */
        if (s_imu.dt_s <= 0.0f || isnan(s_imu.dt_s)) {
            s_imu.dt_s = IMU_DEFAULT_DT;
        }
    }

    s_last_update_us = now_us;
}

/* ------------------------------------------------------------------------ */
/* 4. Sensor read + pre-processing                                          */
/* ------------------------------------------------------------------------ */

static void IMU_App_UpdateSensors(void)
{
    /* Read the newest raw data from IMU and magnetometer. */
    IMU_ReadAccelGyro(&s_imu.gyro_raw, &s_imu.accel_raw);
    IMU_ReadMag(&s_imu.mag_raw);

    /* Remove sensor offsets before filtering and fusion. */
    s_imu.gyro.x = s_imu.gyro_raw.x - s_imu.gyro_offset.x;
    s_imu.gyro.y = s_imu.gyro_raw.y - s_imu.gyro_offset.y;
    s_imu.gyro.z = s_imu.gyro_raw.z - s_imu.gyro_offset.z;

    s_imu.accel.x = s_imu.accel_raw.x - s_imu.accel_offset.x;
    s_imu.accel.y = s_imu.accel_raw.y - s_imu.accel_offset.y;
    s_imu.accel.z = s_imu.accel_raw.z - s_imu.accel_offset.z;

    s_imu.mag_offset.x = 14.50f;
    s_imu.mag_offset.y = -4.70f;
    s_imu.mag_offset.z = 15.00f;

    s_imu.mag.x = s_imu.mag_raw.x - s_imu.mag_offset.x;
    s_imu.mag.y = s_imu.mag_raw.y - s_imu.mag_offset.y;
    s_imu.mag.z = s_imu.mag_raw.z - s_imu.mag_offset.z;


    /* Low-pass filtered accel is used as the gravity observation input. */
    s_imu.accel_filtered.x = IMU_FilterApply(s_imu.accel.x, &s_accel_buf[0], &s_accel_param);
    s_imu.accel_filtered.y = IMU_FilterApply(s_imu.accel.y, &s_accel_buf[1], &s_accel_param);
    s_imu.accel_filtered.z = IMU_FilterApply(s_imu.accel.z, &s_accel_buf[2], &s_accel_param);

    /* Low-pass filtered gyro is used as the angular rate integration input. */
    s_imu.gyro_filtered.x = IMU_FilterApply(s_imu.gyro.x, &s_gyro_buf[0], &s_gyro_param);
    s_imu.gyro_filtered.y = IMU_FilterApply(s_imu.gyro.y, &s_gyro_buf[1], &s_gyro_param);
    s_imu.gyro_filtered.z = IMU_FilterApply(s_imu.gyro.z, &s_gyro_buf[2], &s_gyro_param);

    /* Magnetometer is passed through directly in this version. */
    s_imu.mag_filtered = s_imu.mag;
}

/* ------------------------------------------------------------------------ */
/* 5. Madgwick AHRS fusion                                                  */
/* ------------------------------------------------------------------------ */

static void IMU_App_MadgwickUpdate(void)
{
    /* Convert raw gyro counts to angular rate in deg/s scale used by this project. */
    float gx = s_imu.gyro_filtered.x * GYRO_CALIBRATION_COFF;
    float gy = s_imu.gyro_filtered.y * GYRO_CALIBRATION_COFF;
    float gz = s_imu.gyro_filtered.z * GYRO_CALIBRATION_COFF;
    float ax = s_imu.accel_filtered.x;
    float ay = s_imu.accel_filtered.y;
    float az = s_imu.accel_filtered.z;
    float recip_norm;
    float q0 = s_imu.quat.w;
    float q1 = s_imu.quat.x;
    float q2 = s_imu.quat.y;
    float q3 = s_imu.quat.z;
    float q_dot1;
    float q_dot2;
    float q_dot3;
    float q_dot4;
    float s0;
    float s1;
    float s2;
    float s3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    /* Quaternion derivative from gyro integration only. */
    q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * IMU_DEG2RAD;
    q_dot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * IMU_DEG2RAD;
    q_dot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * IMU_DEG2RAD;
    q_dot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * IMU_DEG2RAD;

#if IMU_USE_MAGNETOMETER
    float mx = s_imu.mag_filtered.x;
    float my = s_imu.mag_filtered.y;
    float mz = s_imu.mag_filtered.z;
    float hx, hy, bx, bz;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recip_norm = IMU_InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        recip_norm = IMU_InvSqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;

        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        float _2bx = 2.0f * bx;
        float _2bz = 2.0f * bz;

        /* 标准的 Madgwick 9轴更新算法替换 sympy 自动推导的多项式 
         * 以避免完全展开的多项式运算失真引发的剧烈震荡 */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _2bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_2bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _2bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_2bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        recip_norm = IMU_InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        q_dot1 -= s_beta * s0;
        q_dot2 -= s_beta * s1;
        q_dot3 -= s_beta * s2;
        q_dot4 -= s_beta * s3;

        s_imu.mag_healthy = (IMU_Pythagorous3(s_imu.mag.x, s_imu.mag.y, s_imu.mag.z) > 0.01f);
    }
#else
    /* 6-DOF Implementation (Accel + Gyro only) - UNCHANGED. */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recip_norm = IMU_InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        s0 = 4.0f * q0 * q2q2 + 2.0f * q2 * ax + 4.0f * q0 * q1q1 - 2.0f * q1 * ay;
        s1 = 4.0f * q1 * q3q3 - 2.0f * q3 * ax + 4.0f * q0q0 * q1 - 2.0f * q0 * ay - 4.0f * q1 + 8.0f * q1 * q1q1 + 8.0f * q1 * q2q2 + 4.0f * q1 * az;
        s2 = 4.0f * q0q0 * q2 + 2.0f * q0 * ax + 4.0f * q2 * q3q3 - 2.0f * q3 * ay - 4.0f * q2 + 8.0f * q2 * q1q1 + 8.0f * q2 * q2q2 + 4.0f * q2 * az;
        s3 = 4.0f * q1q1 * q3 - 2.0f * q1 * ax + 4.0f * q2q2 * q3 - 2.0f * q2 * ay;

        recip_norm = IMU_InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        q_dot1 -= s_beta * s0;
        q_dot2 -= s_beta * s1;
        q_dot3 -= s_beta * s2;
        q_dot4 -= s_beta * s3;

        s_imu.mag_healthy = 0;
    }
#endif

    /* Integrate quaternion derivative using the current loop period. */
    q0 += q_dot1 * s_imu.dt_s;
    q1 += q_dot2 * s_imu.dt_s;
    q2 += q_dot3 * s_imu.dt_s;
    q3 += q_dot4 * s_imu.dt_s;

    /* Keep quaternion length near 1 to prevent numerical drift. */
    recip_norm = IMU_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    s_imu.quat.w = q0 * recip_norm;
    s_imu.quat.x = q1 * recip_norm;
    s_imu.quat.y = q2 * recip_norm;
    s_imu.quat.z = q3 * recip_norm;
}

/* ------------------------------------------------------------------------ */
/* 6. Quaternion -> Euler                                                   */
/* ------------------------------------------------------------------------ */

static void IMU_App_UpdateEuler(void)
{
    float q0 = s_imu.quat.w;
    float q1 = s_imu.quat.x;
    float q2 = s_imu.quat.y;
    float q3 = s_imu.quat.z;
    float sinp;

    /* Pitch limit clamping to prevent asinf() returning NaN */
    sinp = 2.0f * q0 * q2 - 2.0f * q1 * q3;
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }

    /* Export quaternion into user-friendly Euler angles in degrees. */
    s_imu.pitch_deg = asinf(sinp) * IMU_RAD2DEG;
    s_imu.roll_deg  = atan2f(2.0f * q0 * q1 + 2.0f * q2 * q3,
                             1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2) * IMU_RAD2DEG;
    s_imu.yaw_deg   = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3,
                             1.0f - 2.0f * q2 * q2 - 2.0f * q3 * q3) * IMU_RAD2DEG;

    if (s_imu.yaw_deg < 0.0f) {
        /* Convert yaw from [-180, 180] style output to [0, 360). */
        s_imu.yaw_deg += 360.0f;
    }
}

/* ------------------------------------------------------------------------ */
/* 7-9. Public API                                                          */
/* ------------------------------------------------------------------------ */

void IMU_App_Init(void)
{
    /* Clear all state and filter history before first use. */
    memset(&s_imu, 0, sizeof(s_imu));
    memset(s_accel_buf, 0, sizeof(s_accel_buf));
    memset(s_gyro_buf, 0, sizeof(s_gyro_buf));

    /* Bring up the I2C bus and software filters first. */
    ESP_ERROR_CHECK(i2c_master_init());
    IMU_FilterSetCutoff(IMU_SAMPLE_HZ, 15.0f, &s_accel_param);
    IMU_FilterSetCutoff(IMU_SAMPLE_HZ, 50.0f, &s_gyro_param);

    /* Initialize physical sensors and build a stable startup attitude. */
    bmx055_init();
    IMU_App_CalibrateGyro();
    IMU_App_ResetAttitude();
    s_last_update_us = IMU_Micros();
}

void IMU_App_Update(void)
{
    /* One complete IMU cycle. Keep call order stable for deterministic behavior. */
    IMU_App_UpdateDt();
    IMU_App_UpdateSensors();
    IMU_App_MadgwickUpdate();
    IMU_App_UpdateEuler();
}

const IMU_State *IMU_App_GetState(void)
{
    return &s_imu;
}

float IMU_App_GetPitch(void) { return s_imu.pitch_deg; }
float IMU_App_GetRoll(void)  { return s_imu.roll_deg; }
float IMU_App_GetYaw(void)   { return s_imu.yaw_deg; }

/* ------------------------------------------------------------------------ */
/* 11. FreeRTOS task: run the fusion loop, log pitch/roll/yaw periodically  */
/* ------------------------------------------------------------------------ */

static void IMU_App_Task(void *pvParameters)
{
    uint32_t log_period_ms = (uint32_t)(uintptr_t)pvParameters;
    TickType_t last_log_tick = xTaskGetTickCount();
    TickType_t log_period_ticks = pdMS_TO_TICKS(log_period_ms);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    /* Ensure delay is at least 1 tick to prevent continuous polling */
    TickType_t xFrequency = pdMS_TO_TICKS(1000.0f / IMU_SAMPLE_HZ);
    if (xFrequency == 0) {
        xFrequency = 1; 
    }

    while (1) {
        IMU_App_Update();

        if ((xTaskGetTickCount() - last_log_tick) >= log_period_ticks) {
        //    ESP_LOGI(TAG, "Pitch: %8.2f deg | Roll: %8.2f deg | Yaw: %8.2f deg",
        //            s_imu.pitch_deg, s_imu.roll_deg, s_imu.yaw_deg);
        //ESP_LOGI("RAWMAG", "MAG:%f,%f,%f", s_imu.mag.x, s_imu.mag.y, s_imu.mag.z);
        float mag_heading = atan2f(s_imu.mag.y, s_imu.mag.x) * IMU_RAD2DEG;
        if (mag_heading < 0.0f) mag_heading += 360.0f;
        ESP_LOGI("MAGHDG", "heading=%.1f", mag_heading);
        //ESP_LOGI("MAGSHAPE", "%.2f,%.2f", s_imu.mag.y, s_imu.mag.z);
        /* 绕Z轴转一圈期间记录的数据 */
        //ESP_LOGI("MAGCAL_Z", "%.1f,%.1f,%.1f", s_imu.mag.x, s_imu.mag.y, s_imu.mag.z);

/* 绕X轴转一圈期间记录的数据 */
        //ESP_LOGI("MAGCAL_X", "%.1f,%.1f,%.1f", s_imu.mag.x, s_imu.mag.y, s_imu.mag.z);
            last_log_tick = xTaskGetTickCount();
        }

        /* Use vTaskDelayUntil to strictly pace the loop timing */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void IMU_App_StartTask(uint32_t log_period_ms)
{
    xTaskCreate(IMU_App_Task, "imu_app_task", 4096,
                (void *)(uintptr_t)log_period_ms, 5, NULL);
}