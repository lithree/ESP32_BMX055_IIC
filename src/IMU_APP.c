/*
 * File: imu_app.c
 * Brief:
 * IMU application layer ported from a generic MPU6050+compass backend
 * to the BMX055 driver (bmx055_iic.c/.h) on ESP32 / FreeRTOS.
 * Architecture: 6-DOF Madgwick with Decoupled Tilt-Compensated Magnetic Yaw
 */

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "imu_app.h"
#include "bmx_055.h"

#define IMU_USE_MAGNETOMETER 1

#ifndef IMU_ENABLE_HARD_IRON_CALC
#define IMU_ENABLE_HARD_IRON_CALC 0
#endif

#define IMU_FILTER_PI 3.14159265358979323846f

#define IMU_SAMPLE_HZ 200.0f
#define IMU_DEFAULT_DT (1.0f / IMU_SAMPLE_HZ)

/* 1 / 16.384 LSB/deg/s for BMX055 initialized to +/- 2000 deg/s range */
#define GYRO_CALIBRATION_COFF 0.061035f

static const char *TAG = "IMU_APP";

static IMU_State s_imu;
static IMU_ButterParam s_accel_param;
static IMU_ButterParam s_gyro_param;
static IMU_ButterBuffer s_accel_buf[3];
static IMU_ButterBuffer s_gyro_buf[3];
static uint32_t s_last_update_us = 0;

#if IMU_ENABLE_HARD_IRON_CALC
typedef struct
{
    bool active;
    IMU_Vector3f min;
    IMU_Vector3f max;
} IMU_HardIronCalibrator;

static IMU_HardIronCalibrator s_hard_iron_calibrator;
#endif

static bool IMU_App_HardIronEnabled(void)
{
#if IMU_ENABLE_HARD_IRON_CALC
    return s_hard_iron_calibrator.active;
#else
    return false;
#endif
}

static void IMU_App_HardIronCalibratorReset(void)
{
#if IMU_ENABLE_HARD_IRON_CALC
    s_hard_iron_calibrator.active = true;
    s_hard_iron_calibrator.min.x = 0.0f;
    s_hard_iron_calibrator.min.y = 0.0f;
    s_hard_iron_calibrator.min.z = 0.0f;
    s_hard_iron_calibrator.max.x = 0.0f;
    s_hard_iron_calibrator.max.y = 0.0f;
    s_hard_iron_calibrator.max.z = 0.0f;
#else
    (void)0;
#endif
}

static void IMU_App_HardIronCalibratorUpdate(const IMU_Vector3f *mag)
{
#if IMU_ENABLE_HARD_IRON_CALC
    if (!s_hard_iron_calibrator.active)
    {
        return;
    }

    if (s_hard_iron_calibrator.min.x == 0.0f && s_hard_iron_calibrator.min.y == 0.0f && s_hard_iron_calibrator.min.z == 0.0f &&
        s_hard_iron_calibrator.max.x == 0.0f && s_hard_iron_calibrator.max.y == 0.0f && s_hard_iron_calibrator.max.z == 0.0f)
    {
        s_hard_iron_calibrator.min = *mag;
        s_hard_iron_calibrator.max = *mag;
    }
    else
    {
        if (mag->x < s_hard_iron_calibrator.min.x)
            s_hard_iron_calibrator.min.x = mag->x;
        if (mag->y < s_hard_iron_calibrator.min.y)
            s_hard_iron_calibrator.min.y = mag->y;
        if (mag->z < s_hard_iron_calibrator.min.z)
            s_hard_iron_calibrator.min.z = mag->z;

        if (mag->x > s_hard_iron_calibrator.max.x)
            s_hard_iron_calibrator.max.x = mag->x;
        if (mag->y > s_hard_iron_calibrator.max.y)
            s_hard_iron_calibrator.max.y = mag->y;
        if (mag->z > s_hard_iron_calibrator.max.z)
            s_hard_iron_calibrator.max.z = mag->z;
    }
#else
    (void)mag;
#endif
}

static float IMU_App_HardIronGetOffsetX(void)
{
#if IMU_ENABLE_HARD_IRON_CALC
    return (s_hard_iron_calibrator.max.x + s_hard_iron_calibrator.min.x) * 0.5f;
#else
    return 0.0f;
#endif
}

static float IMU_App_HardIronGetOffsetY(void)
{
#if IMU_ENABLE_HARD_IRON_CALC
    return (s_hard_iron_calibrator.max.y + s_hard_iron_calibrator.min.y) * 0.5f;
#else
    return 0.0f;
#endif
}

static float IMU_App_HardIronGetOffsetZ(void)
{
#if IMU_ENABLE_HARD_IRON_CALC
    return (s_hard_iron_calibrator.max.z + s_hard_iron_calibrator.min.z) * 0.5f;
#else
    return 0.0f;
#endif
}

/* --- Calibration Parameters --- */
static float s_mag_offset_x = 5.47f;
static float s_mag_offset_y = 30.60f;
static float s_mag_offset_z = -4.71f;

static float s_mag_scale_x = 1.00f;
static float s_mag_scale_y = 1.00f;
static float s_mag_scale_z = 1.00f;
/* ------------------------------------------------- */
/* 6-DOF Madgwick accelerometer correction weight */
static float s_beta = 0.1f;
/* Magnetometer heading correction factor */
static float s_mag_alpha = 0.0005f;
/* Startup time for mag alpha adjustment */
static uint32_t s_init_time_us = 0;

float yaw_err_obs = 0;

/* Forward declaration */
static void IMU_App_UpdateEuler(void);

/* ------------------------------------------------------------------------ */
/* 1-2. Butterworth low-pass filter                                         */
/* ------------------------------------------------------------------------ */

void IMU_FilterSetCutoff(float sample_hz, float cutoff_hz, IMU_ButterParam *param)
{
    float fr = sample_hz / cutoff_hz;
    float ohm = tanf(IMU_FILTER_PI / fr);
    float c = 1.0f + 2.0f * cosf(IMU_FILTER_PI / 4.0f) * ohm + ohm * ohm;

    param->b0 = (ohm * ohm) / c;
    param->b1 = 2.0f * param->b0;
    param->b2 = param->b0;
    param->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    param->a2 = (1.0f - 2.0f * cosf(IMU_FILTER_PI / 4.0f) * ohm + ohm * ohm) / c;
    param->a0 = 1.0f;
}

float IMU_FilterApply(float input, IMU_ButterBuffer *buf, const IMU_ButterParam *param)
{
    float output = param->b0 * input + param->b1 * buf->x1 + param->b2 * buf->x2 - param->a1 * buf->y1 - param->a2 * buf->y2;

    buf->x2 = buf->x1;
    buf->x1 = input;
    buf->y2 = buf->y1;
    buf->y1 = output;

    return output;
}

/* ------------------------------------------------------------------------ */
/* Small local helpers                                                      */
/* ------------------------------------------------------------------------ */

static inline uint32_t IMU_Micros(void)
{
    return (uint32_t)esp_timer_get_time();
}

static inline void IMU_DelayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void IMU_ReadAccelGyro(IMU_Vector3f *gyro, IMU_Vector3f *accel)
{
    int16_t gx, gy, gz;
    int16_t ax, ay, az;

    if (BMX055_ReadGyroRaw(&gx, &gy, &gz) != ESP_OK)
    {
        gx = gy = gz = 0;
    }
    if (BMX055_ReadAccelRaw(&ax, &ay, &az) != ESP_OK)
    {
        ax = ay = az = 0;
    }

    /* Unify Accel and Gyro to Math Frame: X(Front), Y(Left), Z(Up) */
    gyro->x = (float)gy;
    gyro->y = -(float)gx;
    gyro->z = (float)gz;

    accel->x = (float)ay;
    accel->y = -(float)ax;
    accel->z = (float)az;
}

static void IMU_ReadMag(IMU_Vector3f *mag)
{
    float mx, my, mz;

    if (BMX055_ReadMagCompensated(&mx, &my, &mz) != ESP_OK)
    {
        mx = my = mz = 0.0f;
    }

    mag->x = mx;
    mag->y = my;
    mag->z = mz;
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

    IMU_DelayMs(200);
    for (i = 0; i < 200U; i++)
    {
        IMU_ReadAccelGyro(&gyro_raw, &accel_raw);
        gyro_sum.x += gyro_raw.x;
        gyro_sum.y += gyro_raw.y;
        gyro_sum.z += gyro_raw.z;
        IMU_DelayMs(5);
    }

    s_imu.gyro_offset.x = gyro_sum.x / 200.0f;
    s_imu.gyro_offset.y = gyro_sum.y / 200.0f;
    s_imu.gyro_offset.z = gyro_sum.z / 200.0f;
}

/* ------------------------------------------------------------------------ */
/* 2. Initial attitude from accel                                           */
/* ------------------------------------------------------------------------ */

static void IMU_App_ResetAttitude(void)
{
    float roll_obs, pitch_obs;
    float ax, ay, az;

    IMU_ReadAccelGyro(&s_imu.gyro_raw, &s_imu.accel_raw);

    ax = s_imu.accel_raw.x - s_imu.accel_offset.x;
    ay = s_imu.accel_raw.y - s_imu.accel_offset.y;
    az = s_imu.accel_raw.z - s_imu.accel_offset.z;

    roll_obs = -57.3f * atanf(ax * IMU_InvSqrt(ay * ay + az * az));
    pitch_obs = 57.3f * atanf(ay * IMU_InvSqrt(ax * ax + az * az));

    float pitch = roll_obs * IMU_DEG2RAD;
    float roll = pitch_obs * IMU_DEG2RAD;
    float yaw = 0.0f; // Start at 0, mag will smoothly pull it to North

    s_imu.quat.w = cosf(yaw * 0.5f) * cosf(pitch * 0.5f) * cosf(roll * 0.5f) + sinf(yaw * 0.5f) * sinf(pitch * 0.5f) * sinf(roll * 0.5f);
    s_imu.quat.x = cosf(yaw * 0.5f) * cosf(pitch * 0.5f) * sinf(roll * 0.5f) - sinf(yaw * 0.5f) * sinf(pitch * 0.5f) * cosf(roll * 0.5f);
    s_imu.quat.y = cosf(yaw * 0.5f) * sinf(pitch * 0.5f) * cosf(roll * 0.5f) + sinf(yaw * 0.5f) * cosf(pitch * 0.5f) * sinf(roll * 0.5f);
    s_imu.quat.z = sinf(yaw * 0.5f) * cosf(pitch * 0.5f) * cosf(roll * 0.5f) - cosf(yaw * 0.5f) * sinf(pitch * 0.5f) * sinf(roll * 0.5f);
}

/* ------------------------------------------------------------------------ */
/* 3. Loop timing                                                           */
/* ------------------------------------------------------------------------ */

static void IMU_App_UpdateDt(void)
{
    uint32_t now_us = IMU_Micros();

    if (s_last_update_us == 0U)
    {
        s_imu.dt_s = IMU_DEFAULT_DT;
    }
    else
    {
        s_imu.dt_s = (float)(now_us - s_last_update_us) / 1000000.0f;

        /* Only filter mathematically destructive states. */
        if (s_imu.dt_s <= 0.0f || isnan(s_imu.dt_s))
        {
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
    IMU_ReadAccelGyro(&s_imu.gyro_raw, &s_imu.accel_raw);
    IMU_ReadMag(&s_imu.mag_raw);

    s_imu.gyro.x = s_imu.gyro_raw.x - s_imu.gyro_offset.x;
    s_imu.gyro.y = s_imu.gyro_raw.y - s_imu.gyro_offset.y;
    s_imu.gyro.z = s_imu.gyro_raw.z - s_imu.gyro_offset.z;

    s_imu.accel.x = s_imu.accel_raw.x - s_imu.accel_offset.x;
    s_imu.accel.y = s_imu.accel_raw.y - s_imu.accel_offset.y;
    s_imu.accel.z = s_imu.accel_raw.z - s_imu.accel_offset.z;

    if (IMU_App_HardIronEnabled())
    {
        IMU_App_HardIronCalibratorUpdate(&s_imu.mag_raw);
    }

    /* Apply calibration: (Raw - Offset) * Scale */
    s_imu.mag.x = (s_imu.mag_raw.x - s_imu.mag_offset.x) * s_mag_scale_x;
    s_imu.mag.y = (s_imu.mag_raw.y - s_imu.mag_offset.y) * s_mag_scale_y;
    s_imu.mag.z = (s_imu.mag_raw.z - s_imu.mag_offset.z) * s_mag_scale_z;

    s_imu.accel_filtered.x = IMU_FilterApply(s_imu.accel.x, &s_accel_buf[0], &s_accel_param);
    s_imu.accel_filtered.y = IMU_FilterApply(s_imu.accel.y, &s_accel_buf[1], &s_accel_param);
    s_imu.accel_filtered.z = IMU_FilterApply(s_imu.accel.z, &s_accel_buf[2], &s_accel_param);

    s_imu.gyro_filtered.x = IMU_FilterApply(s_imu.gyro.x, &s_gyro_buf[0], &s_gyro_param);
    s_imu.gyro_filtered.y = IMU_FilterApply(s_imu.gyro.y, &s_gyro_buf[1], &s_gyro_param);
    s_imu.gyro_filtered.z = IMU_FilterApply(s_imu.gyro.z, &s_gyro_buf[2], &s_gyro_param);

    /*
     * ------------------------------------------------------------------------
     * Physical Axis Mapping (from BMX055 Datasheet):
     * ay_chip <->  Bx_chip  (Chip Y-axis Accel corresponds to Chip X-axis Mag)
     * ax_chip <-> -By_chip  (Chip X-axis Accel corresponds to Chip -Y-axis Mag)
     * az_chip <->  Bz_chip  (Chip Z-axis Accel corresponds to Chip Z-axis Mag)
     *
     * Code Mapping to Math Frame (X: Front, Y: Left, Z: Up):
     * accel.x (Front) =  ay_chip = Bx_chip  -> mag_filtered.x should be mag.x
     * accel.y (Left)  = -ax_chip = By_chip  -> mag_filtered.y should be mag.y
     * accel.z (Up)    =  az_chip = Bz_chip  -> mag_filtered.z should be mag.z
     * ------------------------------------------------------------------------
     */
    s_imu.mag_filtered.x = s_imu.mag.x;
    s_imu.mag_filtered.y = s_imu.mag.y;
    s_imu.mag_filtered.z = s_imu.mag.z;
}

/* ------------------------------------------------------------------------ */
/* 5. Decoupled AHRS fusion (6-DOF Madgwick + Tilt-Compensated Yaw)         */
/* ------------------------------------------------------------------------ */

static void IMU_App_MadgwickUpdate(void)
{
    float gx = s_imu.gyro_filtered.x * GYRO_CALIBRATION_COFF * IMU_DEG2RAD;
    float gy = s_imu.gyro_filtered.y * GYRO_CALIBRATION_COFF * IMU_DEG2RAD;
    float gz = s_imu.gyro_filtered.z * GYRO_CALIBRATION_COFF * IMU_DEG2RAD;
    float ax = s_imu.accel_filtered.x;
    float ay = s_imu.accel_filtered.y;
    float az = s_imu.accel_filtered.z;

    float q0 = s_imu.quat.w, q1 = s_imu.quat.x, q2 = s_imu.quat.y, q3 = s_imu.quat.z;
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;

    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    /* Step 1: Pure 6-DOF Madgwick */
    q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    q_dot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    q_dot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
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
    }

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

#if IMU_USE_MAGNETOMETER
    /* Step 2: Decoupled Tilt-Compensated Yaw */
    if (1)
    {
        /* Adjust mag_alpha based on startup time: 0.2 for first 5 seconds, then 0.005 */
        uint32_t elapsed_us = IMU_Micros() - s_init_time_us;
        if (elapsed_us < 5000000U) /* First 5 seconds */
        {
            s_mag_alpha = 0.2f;
        }
        else
        {
            s_mag_alpha = 0.005f;
        }

        /* Extract clean pitch and roll */
        IMU_App_UpdateEuler();

        float mx = s_imu.mag_filtered.x;
        float my = s_imu.mag_filtered.y;
        float mz = s_imu.mag_filtered.z;

        float roll = s_imu.roll_deg * IMU_DEG2RAD;
        float pitch = s_imu.pitch_deg * IMU_DEG2RAD;
        float cr = cosf(roll), sr = sinf(roll);
        float cp = cosf(pitch), sp = sinf(pitch);

        /* Tilt compensation: project 3D magnetic field to horizontal plane */
        float Xh = mx * cp + my * sr * sp + mz * cr * sp;
        float Yh = my * cr - mz * sr;

        if (Xh != 0.0f || Yh != 0.0f)
        {
            /* Note -Yh to fix handedness mismatch */
            float mag_yaw = atan2f(-Yh, Xh) * IMU_RAD2DEG;
            if (mag_yaw < 0.0f)
                mag_yaw += 360.0f;

            /* Step 3: Find shortest error between gyro heading and mag heading */
            float yaw_err = mag_yaw - s_imu.yaw_deg;
            if (yaw_err > 180.0f)
                yaw_err -= 360.0f;
            if (yaw_err < -180.0f)
                yaw_err += 360.0f;

            /* Step 4: Apply smooth correction */
            float correction_rad = (yaw_err * s_mag_alpha) * IMU_DEG2RAD;
            yaw_err_obs = yaw_err;
            float cy = cosf(correction_rad * 0.5f);
            float sy = sinf(correction_rad * 0.5f);

            /* Quaternion multiplication to apply yaw correction */
            float qw = s_imu.quat.w, qx = s_imu.quat.x, qy = s_imu.quat.y, qz = s_imu.quat.z;
            s_imu.quat.w = cy * qw - sy * qz;
            s_imu.quat.x = cy * qx - sy * qy;
            s_imu.quat.y = cy * qy + sy * qx;
            s_imu.quat.z = cy * qz + sy * qw;

            /* Re-normalize */
            recip_norm = IMU_InvSqrt(s_imu.quat.w * s_imu.quat.w + s_imu.quat.x * s_imu.quat.x + s_imu.quat.y * s_imu.quat.y + s_imu.quat.z * s_imu.quat.z);
            s_imu.quat.w *= recip_norm;
            s_imu.quat.x *= recip_norm;
            s_imu.quat.y *= recip_norm;
            s_imu.quat.z *= recip_norm;
        }
    }
#endif
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

    sinp = 2.0f * q0 * q2 - 2.0f * q1 * q3;
    if (sinp > 1.0f)
    {
        sinp = 1.0f;
    }
    else if (sinp < -1.0f)
    {
        sinp = -1.0f;
    }

    s_imu.pitch_deg = asinf(sinp) * IMU_RAD2DEG;
    s_imu.roll_deg = atan2f(2.0f * q0 * q1 + 2.0f * q2 * q3,
                            1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2) *
                     IMU_RAD2DEG;
    s_imu.yaw_deg = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3,
                           1.0f - 2.0f * q2 * q2 - 2.0f * q3 * q3) *
                    IMU_RAD2DEG;

    if (s_imu.yaw_deg < 0.0f)
    {
        s_imu.yaw_deg += 360.0f;
    }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

void IMU_App_RunHardIronCalibration(void)
{
    if (!IMU_App_HardIronEnabled())
    {
        IMU_App_HardIronCalibratorReset();
        ESP_LOGI(TAG, "Hard-iron calibration started. Rotate the board through all orientations.");
    }

    IMU_Vector3f mag;
    IMU_ReadMag(&mag);
    IMU_App_HardIronCalibratorUpdate(&mag);
}

void IMU_App_Init(void)
{
    memset(&s_imu, 0, sizeof(s_imu));
    memset(s_accel_buf, 0, sizeof(s_accel_buf));
    memset(s_gyro_buf, 0, sizeof(s_gyro_buf));

#if IMU_ENABLE_HARD_IRON_CALC
    IMU_App_HardIronCalibratorReset();
#endif

    s_imu.mag_offset.x = s_mag_offset_x;
    s_imu.mag_offset.y = s_mag_offset_y;
    s_imu.mag_offset.z = s_mag_offset_z;

    ESP_ERROR_CHECK(i2c_master_init());
    IMU_FilterSetCutoff(IMU_SAMPLE_HZ, 15.0f, &s_accel_param);
    IMU_FilterSetCutoff(IMU_SAMPLE_HZ, 50.0f, &s_gyro_param);

    bmx055_init();
    IMU_App_CalibrateGyro();
    IMU_App_ResetAttitude();
    s_last_update_us = IMU_Micros();
    s_init_time_us = IMU_Micros();
}

void IMU_App_Update(void)
{
    IMU_App_UpdateDt();
    IMU_App_UpdateSensors();
#if IMU_ENABLE_HARD_IRON_CALC
    IMU_App_RunHardIronCalibration();
#endif
    IMU_App_MadgwickUpdate();
    IMU_App_UpdateEuler();
}

const IMU_State *IMU_App_GetState(void)
{
    return &s_imu;
}

float IMU_App_GetPitch(void) { return s_imu.pitch_deg; }
float IMU_App_GetRoll(void) { return s_imu.roll_deg; }
float IMU_App_GetYaw(void) { return s_imu.yaw_deg; }

/* ------------------------------------------------------------------------ */
/* 11. FreeRTOS task                                                        */
/* ------------------------------------------------------------------------ */

static void IMU_App_Task(void *pvParameters)
{
    uint32_t log_period_ms = (uint32_t)(uintptr_t)pvParameters;
    TickType_t last_log_tick = xTaskGetTickCount();
    TickType_t log_period_ticks = pdMS_TO_TICKS(log_period_ms);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(1000.0f / IMU_SAMPLE_HZ);
    if (xFrequency == 0)
        xFrequency = 1;

    while (1)
    {
        IMU_App_Update();

        if ((xTaskGetTickCount() - last_log_tick) >= log_period_ticks)
        {
            if (IMU_App_HardIronEnabled())
            {
                ESP_LOGI(TAG, "HardIron: X=%8.2f Y=%8.2f Z=%8.2f",
                         IMU_App_HardIronGetOffsetX(),
                         IMU_App_HardIronGetOffsetY(),
                         IMU_App_HardIronGetOffsetZ());
            }
            else
            {
                ESP_LOGI(TAG, "Yaw: %8.2f , P: %8.2f, R: %8.2f, YawErr: %8.2f",
                         s_imu.yaw_deg,
                         s_imu.pitch_deg,
                         s_imu.roll_deg,
                         yaw_err_obs);
            }
            //    ESP_LOGI(TAG, "Pitch: %8.2f deg | Roll: %8.2f deg | Yaw: %8.2f deg",
            //    s_imu.pitch_deg, s_imu.roll_deg, s_imu.yaw_deg);
            // ESP_LOGI("RAWMAG", "MAG:%f,%f,%f", s_imu.mag.x, s_imu.mag.y, s_imu.mag.z);
            float mag_heading = atan2f(s_imu.mag.y, s_imu.mag.x) * IMU_RAD2DEG;
            if (mag_heading < 0.0f)
                mag_heading += 360.0f;
            // ESP_LOGI("MAGHDG", "heading=%.1f", mag_heading);
            // ESP_LOGI("MAGSHAPE", "%.2f,%.2f", s_imu.mag.y, s_imu.mag.z);

            /* Data recorded during Z-axis rotation */
            // ESP_LOGI("MAGCAL_Z", "%.1f,%.1f,%.1f", s_imu.mag.x, s_imu.mag.y, s_imu.mag.z);

            /* Data recorded during X-axis rotation */
            // ESP_LOGI("MAGCAL_X", "%.1f,%.1f,%.1f", s_imu.mag.x, s_imu.mag.y, s_imu.mag.z);
            last_log_tick = xTaskGetTickCount();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void IMU_App_StartTask(uint32_t log_period_ms)
{
    xTaskCreate(IMU_App_Task, "imu_app_task", 4096,
                (void *)(uintptr_t)log_period_ms, 5, NULL);
}