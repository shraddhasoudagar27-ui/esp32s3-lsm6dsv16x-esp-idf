/*
 * smart_hydration_imu.c
 * Smart Water Bottle — IMU-based drinking event detection
 * Device: LSM6DSV16X on ESP32 via I2C
 *
 * Architecture:
 *  - SFLP (on-chip sensor fusion) produces quaternion @ 30 Hz via FIFO
 *  - Quaternion → pitch/roll/yaw on MCU (minimal math)
 *  - Drinking event FSM: IDLE → LIFT → TILT_HOLD → SIP/CHUG → RETURN
 *  - Hardware events (wake-up, 6D, free-fall, tilt) from INT1 GPIO
 *  - FIFO watermark interrupt drives acquisition loop (no polling)
 *  - Low-rate debug task prints live state without bursty output
 *
 * SFLP advantages over raw accel atan2 (per design guide):
 *  - ~1–2° accuracy vs ~5–10° (gyro-compensated, motion-immune)
 *  - Full 3D orientation: pitch + roll + yaw
 *  - <1°/min yaw drift, ~3.5 µA at 15 Hz
 *
 * FIX (2026-04-14): i2c_bus_recover() moved to BEFORE i2c_param_config().
 *   Previously it was called after param_config, and its gpio_reset_pin()
 *   calls at the end undid the peripheral pin assignment, leaving SCL/SDA
 *   as floating inputs when i2c_driver_install ran — causing every I2C
 *   transaction to time-out and WHO_AM_I to never match → "not found".
 *   A 10 ms stabilisation delay after i2c_driver_install is also added so
 *   the bus is settled before the first transaction.
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
/*
 * Using the legacy I2C driver (driver/i2c.h) which is EOL in ESP-IDF v6.0
 * and will be removed in v7.0. To suppress the build warning without
 * migrating yet, add CONFIG_I2C_SUPPRESS_DEPRECATE_WARN=y to sdkconfig.
 * Migration target: driver/i2c_master.h (esp_driver_i2c component).
 */
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── Utility ─────────────────────────────────────────────────────────────── */
#define BIT_U8(n)   ((uint8_t)(1u << (n)))
#define BIT_U16(n)  ((uint16_t)(1u << (n)))
#define RAD2DEG     (180.0f / 3.14159265f)

/* ── I2C & Board Wiring ───────────────────────────────────────────────────── */
#define I2C_NUM         I2C_NUM_0
#define I2C_SDA         8
#define I2C_SCL         9
#define I2C_FREQ_HZ     400000
#define IMU_INT1_GPIO   GPIO_NUM_4

/* ── Device Identity ──────────────────────────────────────────────────────── */
#define LSM6D_ADDR_A            0x6A
#define LSM6D_ADDR_B            0x6B
#define WHO_AM_I_REG            0x0F
#define WHO_AM_I_EXPECTED       0x70   /* LSM6DSV16X */

/* ── Main Page Registers ──────────────────────────────────────────────────── */
#define FUNC_CFG_ACCESS         0x01   /* embedded page gate */
#define FIFO_CTRL1              0x07
#define FIFO_CTRL2              0x08
#define FIFO_CTRL3              0x09
#define FIFO_CTRL4              0x0A
#define INT1_CTRL               0x0D
#define CTRL1_REG               0x10   /* accel ODR / FS */
#define CTRL2_REG               0x11   /* gyro  ODR / FS */
#define CTRL3_REG               0x12   /* BDU, SW-reset, auto-inc */
#define CTRL6_REG               0x15   /* gyro FS */
#define CTRL8_REG               0x17   /* accel FS */
#define FIFO_STATUS1            0x1B
#define FIFO_STATUS2            0x1C
#define ALL_INT_SRC             0x1D
#define WAKE_UP_SRC             0x45
#define TAP_SRC                 0x46
#define D6D_SRC                 0x47
#define EMB_FUNC_STATUS_MAINPAGE 0x49
#define INACTIVITY_DUR          0x54
#define INACTIVITY_THS          0x55
#define TAP_CFG0                0x56
#define TAP_THS_6D              0x59
#define TAP_DUR                 0x5A
#define WAKE_UP_THS             0x5B
#define WAKE_UP_DUR             0x5C
#define FREE_FALL               0x5D
#define MD1_CFG                 0x5E   /* INT1 event routing */
#define FIFO_DATA_OUT_TAG       0x78

/* ── Embedded Function Registers (page-gated via FUNC_CFG_ACCESS) ─────────── */
#define EMB_FUNC_EN_A           0x04   /* bit1=sflp_game_en, bit3=tilt_en */
#define EMB_FUNC_FIFO_EN_A      0x44   /* bit0=game_fifo, bit1=grav_fifo, bit2=gbias_fifo */
#define SFLP_ODR_REG            0x5E   /* bits[5:3] = sflp_game_odr */
#define EMB_FUNC_INIT_A         0x66   /* bit1=sflp_game_init (self-clears) */
#define EMB_FUNC_SRC            0x64   /* tilt event flag (bit4=is_tilt_cl) */

/* ── FIFO Status Bits ─────────────────────────────────────────────────────── */
#define FIFO_STATUS2_DIFF_FIFO_8    BIT_U8(0)
#define FIFO_STATUS2_FULL_IA        BIT_U8(5)
#define FIFO_STATUS2_OVR_IA         BIT_U8(6)
#define FIFO_STATUS2_WTM_IA         BIT_U8(7)

/* ── ALL_INT_SRC Bits ─────────────────────────────────────────────────────── */
#define ALL_INT_SRC_FF_IA           BIT_U8(0)
#define ALL_INT_SRC_WU_IA           BIT_U8(1)
#define ALL_INT_SRC_TAP_IA          BIT_U8(2)
#define ALL_INT_SRC_D6D_IA          BIT_U8(3)
#define ALL_INT_SRC_SLEEP_CHANGE_IA BIT_U8(4)
#define ALL_INT_SRC_EMB_FUNC_IA     BIT_U8(7)

/* ── WAKE_UP_SRC Bits ─────────────────────────────────────────────────────── */
#define WAKE_UP_SRC_Z_WU            BIT_U8(0)
#define WAKE_UP_SRC_Y_WU            BIT_U8(1)
#define WAKE_UP_SRC_X_WU            BIT_U8(2)
#define WAKE_UP_SRC_WU_IA           BIT_U8(3)
#define WAKE_UP_SRC_SLEEP_STATE     BIT_U8(4)
#define WAKE_UP_SRC_FF_IA           BIT_U8(5)
#define WAKE_UP_SRC_SLEEP_CHANGE_IA BIT_U8(6)

/* ── D6D_SRC Bits ────────────────────────────────────────────────────────── */
#define D6D_SRC_XL  BIT_U8(0)
#define D6D_SRC_XH  BIT_U8(1)
#define D6D_SRC_YL  BIT_U8(2)
#define D6D_SRC_YH  BIT_U8(3)
#define D6D_SRC_ZL  BIT_U8(4)
#define D6D_SRC_ZH  BIT_U8(5)
#define D6D_SRC_D6D_IA BIT_U8(6)

/* ── FIFO Data Tags ───────────────────────────────────────────────────────── */
#define FIFO_TAG_EMPTY              0x00
#define FIFO_TAG_GYRO_NC            0x01   /* raw gyro (not used in SFLP mode) */
#define FIFO_TAG_ACCEL_NC           0x02   /* raw accel (not used in SFLP mode) */
#define FIFO_TAG_TIMESTAMP          0x04
#define FIFO_TAG_CFG_CHANGE         0x05
#define FIFO_TAG_SFLP_GAME_ROT      0x13   /* 3×half-float: x,y,z of quaternion */
#define FIFO_TAG_SFLP_GBIAS         0x16   /* 3×half-float: gyro bias estimate */
#define FIFO_TAG_SFLP_GRAVITY       0x17   /* 3×half-float: gravity vector */

/* ── Sensor Configuration ─────────────────────────────────────────────────── */
/* Accel ±4g, Gyro 2000 dps — guide recommendation for drinking gestures */
#define CTRL8_ACCEL_FS_4G           0x01   /* fs_xl[1:0] = 01 */
#define CTRL6_GYRO_FS_2000DPS       0x04   /* fs_g[3:0]  = 0100 */

/* ODR: 30 Hz — low power, sufficient for sip/chug gestures (guide §3 Step 3) */
#define CTRL1_XL_ODR_30HZ           0x04   /* odr_xl[3:0] = 0100 → 30 Hz */
#define CTRL2_G_ODR_30HZ            0x04   /* odr_g[3:0]  = 0100 → 30 Hz */
#define CTRL1_XL_IDLE_1P875HZ       0x01   /* low-power idle */
#define CTRL2_G_IDLE_POWER_DOWN     0x00   /* gyro off in idle */

/* SFLP ODR: 30 Hz (bits[5:3]=001) — guide §step 5d, sip/chug recommended */
#define SFLP_ODR_30HZ               (0x01 << 3)  /* 0b00001000 */

/* FIFO watermark = 32 samples for batch SFLP reads (guide §step 4) */
#define FIFO_WATERMARK_ACTIVE       32
#define FIFO_WATERMARK_IDLE         6

/* FIFO_CTRL3: DISABLE raw batching for SFLP-only mode */
#define FIFO_CTRL3_ACTIVE_30HZ      0x00
#define FIFO_CTRL3_IDLE             0x01   /* accel at 1.875 Hz only */

/* FIFO_CTRL4: Stream (continuous) mode = 0x06; also set SFLP decimation in bits[5:4] */
#define FIFO_CTRL4_STREAM           0x06

/* CTRL3 */
#define CTRL3_SW_RESET              0x01
#define CTRL3_BDU_IF_INC            0x44   /* BDU bit6 + auto-inc bit2 */

/* INT1_CTRL: FIFO watermark → INT1 (bit3) */
#define INT1_CTRL_FIFO_WTM          0x08

/* Hardware event configuration (validated against vendor register map) */
#define INACTIVITY_DUR_CFG          0xB1
#define INACTIVITY_THS_CFG          0x02
#define TAP_CFG0_CFG                0x5F
#define TAP_THS_6D_CFG              0x48
#define TAP_DUR_CFG                 0x7F
#define WAKE_UP_THS_CFG             0x82
#define WAKE_UP_DUR_CFG             0x02
#define FREE_FALL_CFG               0x33
/* MD1_CFG: wake=bit5, 6D=bit4, free-fall=bit3, tap=bit6, sleep=bit7, emb=bit1 */
#define MD1_CFG_EVENTS              0xFC

/* Accel sensitivity: 0.122 mg/LSB @ ±4g (per ST datasheet table 3) */
#define ACCEL_SENS_G_PER_LSB        0.000122f
/* Gyro sensitivity: 70 mdps/LSB @ 2000 dps */
#define GYRO_SENS_DPS_PER_LSB       0.070f

/* ── Application Thresholds ───────────────────────────────────────────────── */
#define LIFT_PITCH_DEG          15.0f   /* min pitch to count as "lifted" */
#define SIP_PITCH_DEG           30.0f   /* pitch ≥ 30° sustained → sip */
#define CHUG_PITCH_DEG          60.0f   /* pitch ≥ 60° → chug */
#define SIP_HOLD_MS             500u    /* must sustain tilt ≥ 500 ms → sip */
#define CHUG_HOLD_MS            1500u   /* must sustain ≥ 1.5 s → chug */
#define RETURN_PITCH_DEG        10.0f   /* pitch < 10° → bottle returned */
#define RETURN_DEBOUNCE_MS      300u

/* ── Task & Buffer Config ─────────────────────────────────────────────────── */
#define IMU_RING_CAPACITY           256
#define IMU_TASK_STACK_WORDS        4096
#define IMU_TASK_PRIORITY           5
#define IMU_DEBUG_TASK_STACK_WORDS  3072
#define IMU_DEBUG_TASK_PRIORITY     3
#define LIVE_PRINT_PERIOD_MS        500

/* ── Data Types ───────────────────────────────────────────────────────────── */

typedef struct {
    float qw, qx, qy, qz;      /* unit quaternion (WXYZ) */
    float pitch, roll, yaw;     /* Euler angles in degrees */
    float gx, gy, gz;           /* gyro bias (dps) — from SFLP */
    float gravity_x, gravity_y, gravity_z; /* gravity vector (g) */
    uint32_t timestamp_ms;
} imu_sample_t;

typedef struct {
    imu_sample_t samples[IMU_RING_CAPACITY];
    uint16_t head, tail, count;
    uint32_t overflow_count;
} imu_ring_t;

typedef struct {
    uint32_t fifo_interrupts;
    uint32_t fifo_batches;
    uint32_t fifo_words;
    uint32_t fifo_overruns;
    uint32_t quaternion_frames;
    uint32_t wake_events;
    uint32_t sleep_change_events;
    uint32_t free_fall_events;
    uint32_t sixd_events;
    uint32_t single_tap_events;
    uint32_t double_tap_events;
    uint32_t hw_tilt_events;
    uint32_t sip_count;
    uint32_t chug_count;
} imu_stats_t;

typedef enum {
    IMU_PROFILE_ACTIVE = 0,
    IMU_PROFILE_IDLE,
} imu_profile_t;

/* Drinking detection finite state machine */
typedef enum {
    DRINK_STATE_IDLE = 0,
    DRINK_STATE_LIFTED,         /* bottle lifted, watching pitch */
    DRINK_STATE_TILTING,        /* pitch > SIP threshold, timing */
    DRINK_STATE_SIP_CONFIRMED,  /* sip event logged */
    DRINK_STATE_CHUG_CONFIRMED, /* chug event logged */
} drink_state_t;

/* ── Globals ──────────────────────────────────────────────────────────────── */
static uint8_t          g_lsm6d_addr        = LSM6D_ADDR_A;
static TaskHandle_t     g_imu_task_handle   = NULL;
static imu_ring_t       g_ring              = {0};
static imu_stats_t      g_stats             = {0};
static imu_profile_t    g_profile           = IMU_PROFILE_ACTIVE;
static portMUX_TYPE     g_ring_lock         = portMUX_INITIALIZER_UNLOCKED;

/* Drinking FSM state */
static drink_state_t    g_drink_state       = DRINK_STATE_IDLE;
static uint32_t         g_tilt_start_ms     = 0;
static float            g_peak_pitch        = 0.0f;
static uint32_t         g_total_sip_events  = 0;
static uint32_t         g_total_chug_events = 0;

/* Latest orientation snapshot (written by IMU task, read by debug task) */
static volatile float   g_live_pitch        = 0.0f;
static volatile float   g_live_roll         = 0.0f;
static volatile float   g_live_yaw          = 0.0f;
static volatile float   g_live_gx           = 0.0f;
static volatile float   g_live_gy           = 0.0f;
static volatile float   g_live_gz           = 0.0f;
static float            g_idle_qw           = 1.0f;
static float            g_idle_qx           = 0.0f;
static float            g_idle_qy           = 0.0f;
static float            g_idle_qz           = 0.0f;
static bool             g_idle_orientation_valid = false;

/* ── Low-level I/O ────────────────────────────────────────────────────────── */

static esp_err_t reg_write(uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    return i2c_master_write_to_device(I2C_NUM, g_lsm6d_addr,
                                      payload, sizeof(payload), pdMS_TO_TICKS(100));
}

static esp_err_t reg_read(uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(I2C_NUM, g_lsm6d_addr,
                                        &reg, 1, value, 1, pdMS_TO_TICKS(100));
}

static esp_err_t reg_burst(uint8_t start_reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_NUM, g_lsm6d_addr,
                                        &start_reg, 1, buf, len, pdMS_TO_TICKS(100));
}

static inline int16_t to_int16(uint8_t lo, uint8_t hi)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

/* ── Half-float → float (IEEE 754 half-precision decoder) ────────────────── */
static float half_to_float(uint16_t h)
{
    const uint32_t sign     = (uint32_t)(h & 0x8000u) << 16;
    const uint32_t exponent = (uint32_t)(h & 0x7C00u);
    const uint32_t mantissa = (uint32_t)(h & 0x03FFu);
    uint32_t f;

    if (exponent == 0u) {
        /* subnormal */
        if (mantissa == 0u) {
            f = sign;
        } else {
            uint32_t m = mantissa;
            uint32_t e = 0u;
            while ((m & 0x0400u) == 0u) { m <<= 1; e++; }
            f = sign | (((127u - 14u - e) << 23) & 0x7F800000u) | ((m & 0x03FFu) << 13);
        }
    } else if (exponent == 0x7C00u) {
        /* inf / NaN */
        f = sign | 0x7F800000u | (mantissa << 13);
    } else {
        f = sign | (((exponent >> 10) + (127u - 15u)) << 23) | (mantissa << 13);
    }

    float result;
    memcpy(&result, &f, sizeof(result));
    return result;
}

/*
 * sflp2q: Convert 6-byte FIFO payload (3×half-float x,y,z) to full WXYZ quaternion.
 * The FIFO outputs only x,y,z; w is recovered from the unit-quaternion constraint:
 *   w = sqrt(1 - x²- y² - z²)   (guide §4.3)
 * Guard: if sum-of-squares > 1 due to rounding, clamp w to 0.
 */
static void sflp2q(const uint8_t *data, float *qw, float *qx, float *qy, float *qz)
{
    const uint16_t hx = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
    const uint16_t hy = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
    const uint16_t hz = (uint16_t)(data[4] | ((uint16_t)data[5] << 8));

    *qx = half_to_float(hx);
    *qy = half_to_float(hy);
    *qz = half_to_float(hz);

    const float sumsq = (*qx)*(*qx) + (*qy)*(*qy) + (*qz)*(*qz);
    *qw = (sumsq >= 1.0f) ? 0.0f : sqrtf(1.0f - sumsq);
}

/*
 * quaternion_to_euler: Convert WXYZ quaternion → Euler angles (degrees).
 * Using the aerospace/NED convention from guide §2.2:
 *   pitch = -asin(2(xz - wy))          — tilt angle
 *   roll  = atan2(2(wx + yz), w²-x²-y²+z²)  — tilt direction
 *   yaw   = atan2(2(xy + wz), w²+x²-y²-z²)  — heading
 */
static void quaternion_to_euler(float w, float x, float y, float z,
                                 float *pitch, float *roll, float *yaw)
{
    *pitch = -asinf(2.0f * (x*z - w*y)) * RAD2DEG;
    *roll  =  atan2f(2.0f*(w*x + y*z),  w*w - x*x - y*y + z*z) * RAD2DEG;
    *yaw   =  atan2f(2.0f*(x*y + w*z),  w*w + x*x - y*y - z*z) * RAD2DEG;
}

static void quaternion_conjugate(float w, float x, float y, float z,
                                 float *rw, float *rx, float *ry, float *rz)
{
    *rw = w;
    *rx = -x;
    *ry = -y;
    *rz = -z;
}

static void quaternion_multiply(float aw, float ax, float ay, float az,
                                float bw, float bx, float by, float bz,
                                float *rw, float *rx, float *ry, float *rz)
{
    *rw = aw*bw - ax*bx - ay*by - az*bz;
    *rx = aw*bx + ax*bw + ay*bz - az*by;
    *ry = aw*by - ax*bz + ay*bw + az*bx;
    *rz = aw*bz + ax*by - ay*bx + az*bw;
}

static void capture_idle_baseline(float qw, float qx, float qy, float qz)
{
    g_idle_qw = qw;
    g_idle_qx = qx;
    g_idle_qy = qy;
    g_idle_qz = qz;
    g_idle_orientation_valid = true;
}

/* ── Ring Buffer ──────────────────────────────────────────────────────────── */

static void imu_ring_push(const imu_sample_t *s)
{
    portENTER_CRITICAL(&g_ring_lock);
    if (g_ring.count == IMU_RING_CAPACITY) {
        g_ring.tail = (uint16_t)((g_ring.tail + 1u) % IMU_RING_CAPACITY);
        g_ring.overflow_count++;
        g_ring.count--;
    }
    g_ring.samples[g_ring.head] = *s;
    g_ring.head = (uint16_t)((g_ring.head + 1u) % IMU_RING_CAPACITY);
    g_ring.count++;
    portEXIT_CRITICAL(&g_ring_lock);
}

/* ── Initialisation ───────────────────────────────────────────────────────── */

/*
 * i2c_bus_recover: Toggle SCL up to 9 times to unstick a device that is
 * holding SDA low from an interrupted previous transaction (e.g. after a
 * power-cycle or watchdog reset mid-transfer).  Follows the I2C spec
 * §3.1.16 bus-clear procedure.  Must be called BEFORE i2c_param_config
 * because it drives the pins directly via gpio, and must complete before
 * the I2C peripheral takes ownership of the pins.
 *
 * After toggling, sends a STOP condition (SDA low→high while SCL high)
 * so any device stuck in a DATA byte releases the bus.
 */
static void i2c_bus_recover(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << I2C_SCL) | (1ULL << I2C_SDA),
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    gpio_set_level(I2C_SCL, 1);
    gpio_set_level(I2C_SDA, 1);
    esp_rom_delay_us(10);

    if (gpio_get_level(I2C_SDA) == 0) {
        for (int i = 0; i < 9; i++) {
            gpio_set_level(I2C_SCL, 0); esp_rom_delay_us(5);
            gpio_set_level(I2C_SCL, 1); esp_rom_delay_us(5);
            if (gpio_get_level(I2C_SDA)) break;
        }
        /* Issue STOP: SDA low→high while SCL high */
        gpio_set_level(I2C_SDA, 0); esp_rom_delay_us(5);
        gpio_set_level(I2C_SCL, 1); esp_rom_delay_us(5);
        gpio_set_level(I2C_SDA, 1); esp_rom_delay_us(5);
    }

    gpio_reset_pin(I2C_SCL);
    gpio_reset_pin(I2C_SDA);
}

static void i2c_init(void)
{
    /*
     * FIX: Bus recovery MUST happen before i2c_param_config().
     *
     * The previous order was:
     *   1. i2c_param_config()   — assigns SCL/SDA to the I2C peripheral
     *   2. i2c_bus_recover()    — at the end calls gpio_reset_pin() on both
     *                             pins, silently un-assigning them from I2C
     *   3. i2c_driver_install() — installs driver on pins that are now reset
     *                             floating inputs → all transactions time-out
     *                             → WHO_AM_I never matches → "not found"
     *
     * Correct order: recover first (pins are free GPIO), then hand them
     * to the I2C peripheral via param_config + driver_install.
     */
    i2c_bus_recover();

    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA,
        .scl_io_num       = I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

    /*
     * Allow the bus and pull-ups to settle after the driver takes ownership
     * of the pins.  Without this, the very first transaction on a cold boot
     * can see a false NACK while the open-drain lines charge up.
     */
    vTaskDelay(pdMS_TO_TICKS(10));
}

static esp_err_t detect_lsm6d_address(void)
{
    for (uint8_t addr = LSM6D_ADDR_A; addr <= LSM6D_ADDR_B; ++addr) {
        uint8_t who = 0;
        esp_err_t err = i2c_master_write_read_device(
            I2C_NUM, addr, (uint8_t[]){WHO_AM_I_REG}, 1, &who, 1, pdMS_TO_TICKS(100));
        if (err == ESP_OK && who == WHO_AM_I_EXPECTED) {
            g_lsm6d_addr = addr;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

/*
 * imu_configure_sflp: Set up embedded function registers for SFLP game rotation.
 * Sequence per guide §3 Step 5:
 *   5a. Enter embedded register page (FUNC_CFG_ACCESS = 0x80)
 *   5b. Enable sflp_game_en (EMB_FUNC_EN_A bit1) + tilt_en (bit3)
 *   5c. Route quaternion+gravity to FIFO (EMB_FUNC_FIFO_EN_A bits 0,1)
 *   5d. Set SFLP ODR = 30 Hz (SFLP_ODR_REG bits[5:3]=001)
 *   5e. Trigger init (EMB_FUNC_INIT_A bit1, self-clears)
 *   5f. Exit embedded register page
 */
static esp_err_t imu_configure_sflp(void)
{
    esp_err_t err;
    uint8_t chk = 0;

    /* 5a — enter embedded function register bank + VERIFY */
    err = reg_write(FUNC_CFG_ACCESS, 0x80);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(2));

    err = reg_read(FUNC_CFG_ACCESS, &chk);
    if (err != ESP_OK) goto close_page;
    if ((chk & 0x80) == 0) {
        err = ESP_FAIL;
        goto close_page;
    }

    /* 5b — enable SFLP game rotation + hardware tilt + VERIFY/RETRY */
    err = reg_write(EMB_FUNC_EN_A, BIT_U8(1) | BIT_U8(3));
    if (err != ESP_OK) goto close_page;

    err = reg_read(EMB_FUNC_EN_A, &chk);
    if (err != ESP_OK) goto close_page;
    if ((chk & BIT_U8(1)) == 0) {
        err = reg_write(EMB_FUNC_EN_A, BIT_U8(1) | BIT_U8(3));
        if (err != ESP_OK) goto close_page;
    }

    /* 5c — route quaternion and gravity vector to FIFO */
    err = reg_write(EMB_FUNC_FIFO_EN_A, BIT_U8(0) | BIT_U8(1));
    if (err != ESP_OK) goto close_page;

    /* 5d — SFLP ODR = 30 Hz: sflp_game_odr[5:3]=001 */
    err = reg_write(SFLP_ODR_REG, SFLP_ODR_30HZ);
    if (err != ESP_OK) goto close_page;

    /* 5e — initialise SFLP engine (bit1 self-clears) */
    err = reg_write(EMB_FUNC_INIT_A, BIT_U8(1));
    if (err != ESP_OK) goto close_page;

    /* Verify SFLP is enabled before closing page */
    err = reg_read(EMB_FUNC_EN_A, &chk);
    if (err != ESP_OK) goto close_page;
    if (chk != (BIT_U8(1) | BIT_U8(3))) {
        err = ESP_FAIL;
        goto close_page;
    }

close_page:
    /* 5f — always close embedded page regardless of error */
    {
        esp_err_t close_err = reg_write(FUNC_CFG_ACCESS, 0x00);
        if (err == ESP_OK) err = close_err;
    }
    return err;
}

static esp_err_t imu_apply_profile(imu_profile_t profile)
{
    if (profile == g_profile) return ESP_OK;

    esp_err_t err;
    if (profile == IMU_PROFILE_ACTIVE) {
        err  = reg_write(FIFO_CTRL1, FIFO_WATERMARK_ACTIVE);
        err |= reg_write(CTRL1_REG,  CTRL1_XL_ODR_30HZ);
        err |= reg_write(CTRL2_REG,  CTRL2_G_ODR_30HZ);
        err |= reg_write(FIFO_CTRL3, FIFO_CTRL3_ACTIVE_30HZ);
        if (err == ESP_OK) g_profile = IMU_PROFILE_ACTIVE;
        return err;
    }

    /* IDLE: gyro off, accel at 1.875 Hz, SFLP naturally pauses */
    err  = reg_write(FIFO_CTRL1, FIFO_WATERMARK_IDLE);
    err |= reg_write(FIFO_CTRL3, FIFO_CTRL3_IDLE);
    err |= reg_write(CTRL2_REG,  CTRL2_G_IDLE_POWER_DOWN);
    err |= reg_write(CTRL1_REG,  CTRL1_XL_IDLE_1P875HZ);
    if (err == ESP_OK) g_profile = IMU_PROFILE_IDLE;
    return err;
}

static esp_err_t imu_configure(void)
{
    esp_err_t err;

    /* Step 1: Software reset.
     *
     * The reset bit (sw_reset, CTRL3 bit0) self-clears once the device is
     * ready.  Poll for it rather than blindly delaying.  If the first write
     * times out the bus may still be recovering — retry once after a short
     * pause before giving up.
     */
    err = reg_write(CTRL3_REG, CTRL3_SW_RESET);
    if (err == ESP_ERR_TIMEOUT) {
        vTaskDelay(pdMS_TO_TICKS(50));
        err = reg_write(CTRL3_REG, CTRL3_SW_RESET);
    }
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(5));

    /* Poll until sw_reset bit clears (typ <1 ms, max 50 ms per datasheet) */
    {
        uint8_t ctrl3 = CTRL3_SW_RESET;
        for (int i = 0; i < 50 && (ctrl3 & CTRL3_SW_RESET); i++) {
            vTaskDelay(pdMS_TO_TICKS(2));
            if (reg_read(CTRL3_REG, &ctrl3) != ESP_OK) ctrl3 = CTRL3_SW_RESET;
        }
        if (ctrl3 & CTRL3_SW_RESET) return ESP_ERR_TIMEOUT;
    }

    /* Step 1 cont: BDU + auto-increment */
    err = reg_write(CTRL3_REG, CTRL3_BDU_IF_INC);
    if (err != ESP_OK) return err;

    /* Step 2: Full-scale ranges — ±4g accel, 2000 dps gyro (guide §3 Step 2) */
    err  = reg_write(CTRL8_REG, CTRL8_ACCEL_FS_4G);
    err |= reg_write(CTRL6_REG, CTRL6_GYRO_FS_2000DPS);
    if (err != ESP_OK) return err;

    /* Step 3: ODR — 30 Hz for both axes (set BEFORE enabling SFLP per guide) */
    err  = reg_write(CTRL1_REG, CTRL1_XL_ODR_30HZ);
    err |= reg_write(CTRL2_REG, CTRL2_G_ODR_30HZ);
    if (err != ESP_OK) return err;

    /* Step 4: FIFO in continuous/stream mode with watermark */
    err  = reg_write(FIFO_CTRL1, FIFO_WATERMARK_ACTIVE);
    err |= reg_write(FIFO_CTRL2, 0x00);
    err |= reg_write(FIFO_CTRL3, FIFO_CTRL3_ACTIVE_30HZ);
    err |= reg_write(FIFO_CTRL4, FIFO_CTRL4_STREAM);
    if (err != ESP_OK) return err;

    /* INT1: FIFO watermark interrupt */
    err = reg_write(INT1_CTRL, INT1_CTRL_FIFO_WTM);
    if (err != ESP_OK) return err;

    /* Hardware event configuration */
    err  = reg_write(INACTIVITY_DUR, INACTIVITY_DUR_CFG);
    err |= reg_write(INACTIVITY_THS, INACTIVITY_THS_CFG);
    err |= reg_write(TAP_CFG0,       TAP_CFG0_CFG);
    err |= reg_write(TAP_THS_6D,     TAP_THS_6D_CFG);
    err |= reg_write(TAP_DUR,        TAP_DUR_CFG);
    err |= reg_write(WAKE_UP_THS,    WAKE_UP_THS_CFG);
    err |= reg_write(WAKE_UP_DUR,    WAKE_UP_DUR_CFG);
    err |= reg_write(FREE_FALL,      FREE_FALL_CFG);
    err |= reg_write(MD1_CFG,        MD1_CFG_EVENTS);
    if (err != ESP_OK) return err;

    /* Step 5: Enable SFLP in embedded function registers */
    err = imu_configure_sflp();
    if (err != ESP_OK) return err;

    /* Force profile transition to ACTIVE */
    g_profile = IMU_PROFILE_IDLE;
    err = imu_apply_profile(IMU_PROFILE_ACTIVE);
    if (err != ESP_OK) return err;

    /* Re-run SFLP config after profile switch */
    return imu_configure_sflp();
}

/* ── FIFO work pending check ──────────────────────────────────────────────── */

static bool imu_fifo_work_pending(void)
{
    uint8_t s[2] = {0};
    if (reg_burst(FIFO_STATUS1, s, 2u) != ESP_OK)
        return gpio_get_level(IMU_INT1_GPIO) != 0;

    const uint16_t words =
        (uint16_t)s[0] | ((uint16_t)(s[1] & FIFO_STATUS2_DIFF_FIFO_8) << 8);

    return (words != 0u)
        || ((s[1] & FIFO_STATUS2_WTM_IA) != 0u)
        || (gpio_get_level(IMU_INT1_GPIO) != 0);
}

/* ── Drinking Event FSM ───────────────────────────────────────────────────── */

/*
 * Called once per committed quaternion frame.
 * State transitions:
 *   IDLE       → LIFTED      when pitch > LIFT_PITCH_DEG
 *   LIFTED     → TILTING     when pitch > SIP_PITCH_DEG (start timing)
 *   TILTING    → SIP/CHUG    when pitch sustained past threshold duration
 *   TILTING    → LIFTED      when pitch drops back below SIP threshold
 *   SIP/CHUG   → IDLE        when pitch < RETURN_PITCH_DEG
 *   Any state  → IDLE        when SLEEP_CHANGE event fires (handled externally)
 */
static void drinking_fsm_update(float pitch)
{
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    const float abs_pitch = fabsf(pitch);

    switch (g_drink_state) {

    case DRINK_STATE_IDLE:
        if (abs_pitch > LIFT_PITCH_DEG) {
            g_drink_state = DRINK_STATE_LIFTED;
            g_peak_pitch  = abs_pitch;
        }
        break;

    case DRINK_STATE_LIFTED:
        if (abs_pitch > g_peak_pitch) g_peak_pitch = abs_pitch;
        if (abs_pitch > SIP_PITCH_DEG) {
            g_drink_state   = DRINK_STATE_TILTING;
            g_tilt_start_ms = now_ms;
        } else if (abs_pitch < RETURN_PITCH_DEG) {
            g_drink_state = DRINK_STATE_IDLE;
        }
        break;

    case DRINK_STATE_TILTING: {
        if (abs_pitch > g_peak_pitch) g_peak_pitch = abs_pitch;
        const uint32_t elapsed = now_ms - g_tilt_start_ms;

        if (abs_pitch < SIP_PITCH_DEG) {
            g_drink_state = DRINK_STATE_LIFTED;
            break;
        }

        if (abs_pitch >= CHUG_PITCH_DEG && elapsed >= CHUG_HOLD_MS) {
            g_drink_state = DRINK_STATE_CHUG_CONFIRMED;
            g_stats.chug_count++;
            g_total_chug_events++;
        } else if (abs_pitch >= SIP_PITCH_DEG && elapsed >= SIP_HOLD_MS) {
            g_drink_state = DRINK_STATE_SIP_CONFIRMED;
            g_stats.sip_count++;
            g_total_sip_events++;
        }
        break;
    }

    case DRINK_STATE_SIP_CONFIRMED:
    case DRINK_STATE_CHUG_CONFIRMED:
        if (abs_pitch < RETURN_PITCH_DEG) {
            g_drink_state = DRINK_STATE_IDLE;
            g_peak_pitch  = 0.0f;
        }
        break;

    default:
        g_drink_state = DRINK_STATE_IDLE;
        break;
    }
}

/* ── Hardware Event Handler ───────────────────────────────────────────────── */

static bool imu_handle_event_sources(void)
{
    bool hw_seen = false;
    uint8_t all_int = 0;

    if (reg_read(ALL_INT_SRC, &all_int) != ESP_OK) return false;

    if (all_int & (ALL_INT_SRC_WU_IA | ALL_INT_SRC_FF_IA | ALL_INT_SRC_SLEEP_CHANGE_IA)) {
        uint8_t wus = 0;
        if (reg_read(WAKE_UP_SRC, &wus) == ESP_OK) {
            if (wus & WAKE_UP_SRC_SLEEP_CHANGE_IA) {
                g_stats.sleep_change_events++;
                hw_seen = true;
                if (wus & WAKE_UP_SRC_SLEEP_STATE) {
                    imu_apply_profile(IMU_PROFILE_IDLE);
                    g_drink_state = DRINK_STATE_IDLE;
                } else {
                    imu_apply_profile(IMU_PROFILE_ACTIVE);
                    imu_configure_sflp();
                }
            }
            if (wus & WAKE_UP_SRC_WU_IA) {
                g_stats.wake_events++;
                hw_seen = true;
                if (g_profile != IMU_PROFILE_ACTIVE) {
                    imu_apply_profile(IMU_PROFILE_ACTIVE);
                    imu_configure_sflp();
                }
            }
            if (wus & WAKE_UP_SRC_FF_IA) {
                g_stats.free_fall_events++;
                hw_seen = true;
                g_drink_state = DRINK_STATE_IDLE;
            }
        }
    }

    if (all_int & ALL_INT_SRC_D6D_IA) {
        uint8_t d6d = 0;
        if (reg_read(D6D_SRC, &d6d) == ESP_OK && (d6d & D6D_SRC_D6D_IA)) {
            g_stats.sixd_events++;
            hw_seen = true;
        }
    }

    if (all_int & ALL_INT_SRC_TAP_IA) {
        uint8_t tap = 0;
        if (reg_read(TAP_SRC, &tap) == ESP_OK && (tap & BIT_U8(6))) {
            hw_seen = true;
            if (tap & BIT_U8(4))      g_stats.double_tap_events++;
            else if (tap & BIT_U8(5)) g_stats.single_tap_events++;
        }
    }

    if (all_int & ALL_INT_SRC_EMB_FUNC_IA) {
        uint8_t emb = 0;
        if (reg_read(EMB_FUNC_STATUS_MAINPAGE, &emb) == ESP_OK && (emb & BIT_U8(4))) {
            g_stats.hw_tilt_events++;
            hw_seen = true;
        }
    }

    return hw_seen;
}

/* ── FIFO Drain — SFLP Quaternion Path ───────────────────────────────────── */

static void imu_drain_fifo(void)
{
    enum { BURST_WORDS = 16 };
    uint8_t burst[BURST_WORDS * 7];

    while (true) {
        uint8_t st[2] = {0};
        if (reg_burst(FIFO_STATUS1, st, 2u) != ESP_OK) return;

        const uint16_t words =
            (uint16_t)st[0] | ((uint16_t)(st[1] & FIFO_STATUS2_DIFF_FIFO_8) << 8);

        if (words == 0u) return;

        g_stats.fifo_batches++;
        g_stats.fifo_words += words;

        if (st[1] & FIFO_STATUS2_OVR_IA) g_stats.fifo_overruns++;

        static float pend_qw = 1.0f, pend_qx = 0.0f,
                     pend_qy = 0.0f, pend_qz = 0.0f;
        static float pend_gx = 0.0f, pend_gy = 0.0f, pend_gz = 0.0f;

        uint16_t idx = 0;
        while (idx < words) {
            uint16_t chunk = (uint16_t)(words - idx);
            if (chunk > BURST_WORDS) chunk = BURST_WORDS;

            if (reg_burst(FIFO_DATA_OUT_TAG, burst, (size_t)chunk * 7u) != ESP_OK) return;

            for (uint16_t j = 0; j < chunk; ++j) {
                const uint8_t *w   = &burst[j * 7u];
                const uint8_t  tag = (w[0] >> 3) & 0x1Fu;
                const uint8_t *p   = &w[1];

                if (tag == FIFO_TAG_EMPTY) return;

                switch (tag) {

                case FIFO_TAG_SFLP_GAME_ROT:
                    /*
                     * Primary data path: on-chip sensor fusion quaternion.
                     * 3×half-float (x, y, z); w recovered from unit constraint.
                     * FIFO order is XYZW; reorder to WXYZ for our convention.
                     * (guide §4.1, §4.2, §4.3)
                     */
                    sflp2q(p, &pend_qw, &pend_qx, &pend_qy, &pend_qz);
                    g_stats.quaternion_frames++;

                    {
                        float pitch, roll, yaw;
                        float rel_qw = pend_qw, rel_qx = pend_qx,
                              rel_qy = pend_qy, rel_qz = pend_qz;

                        if (g_idle_orientation_valid) {
                            float inv_qw, inv_qx, inv_qy, inv_qz;
                            quaternion_conjugate(g_idle_qw, g_idle_qx, g_idle_qy, g_idle_qz,
                                                 &inv_qw, &inv_qx, &inv_qy, &inv_qz);
                            quaternion_multiply(inv_qw, inv_qx, inv_qy, inv_qz,
                                                pend_qw, pend_qx, pend_qy, pend_qz,
                                                &rel_qw, &rel_qx, &rel_qy, &rel_qz);
                        } else if (g_drink_state == DRINK_STATE_IDLE) {
                            capture_idle_baseline(pend_qw, pend_qx, pend_qy, pend_qz);
                        }

                        quaternion_to_euler(rel_qw, rel_qx, rel_qy, rel_qz,
                                            &pitch, &roll, &yaw);

                        g_live_pitch = pitch;
                        g_live_roll  = roll;
                        g_live_yaw   = yaw;

                        {
                            uint32_t ts = (uint32_t)(esp_timer_get_time() / 1000ULL);
                            printf("%"PRIu32",%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f\n",
                                   ts, rel_qw, rel_qx, rel_qy, rel_qz, pitch, roll, yaw);
                        }

                        drinking_fsm_update(pitch);

                        imu_sample_t s = {
                            .qw = rel_qw, .qx = rel_qx,
                            .qy = rel_qy, .qz = rel_qz,
                            .pitch = pitch, .roll = roll, .yaw = yaw,
                            .gx = pend_gx, .gy = pend_gy, .gz = pend_gz,
                            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
                        };
                        imu_ring_push(&s);
                    }
                    break;

                case FIFO_TAG_SFLP_GBIAS:
                    /*
                     * Gyroscope bias estimate from SFLP filter.
                     * Stored for diagnostics; not used in angle computation
                     * (the on-chip SFLP already applies it internally).
                     */
                    pend_gx = half_to_float((uint16_t)(p[0] | ((uint16_t)p[1] << 8)));
                    pend_gy = half_to_float((uint16_t)(p[2] | ((uint16_t)p[3] << 8)));
                    pend_gz = half_to_float((uint16_t)(p[4] | ((uint16_t)p[5] << 8)));
                    g_live_gx = pend_gx;
                    g_live_gy = pend_gy;
                    g_live_gz = pend_gz;
                    break;

                case FIFO_TAG_SFLP_GRAVITY:
                    /* Gravity vector — available for future use */
                    break;

                case FIFO_TAG_GYRO_NC:
                case FIFO_TAG_ACCEL_NC: {
                    static uint32_t s_raw_warn = 0;
                    s_raw_warn++;
                    (void)s_raw_warn;  /* suppress unused-variable warning */
                    break;
                }

                case FIFO_TAG_TIMESTAMP:
                case FIFO_TAG_CFG_CHANGE:
                    break;

                default:
                    break;
                }
            }
            idx = (uint16_t)(idx + chunk);
        }

        /* Re-arm: WTM_IA is a level signal — re-notify if still asserted */
        uint8_t post = 0;
        if (reg_read(FIFO_STATUS2, &post) == ESP_OK &&
            (post & FIFO_STATUS2_WTM_IA) != 0u) {
            xTaskNotify(g_imu_task_handle, 1u, eIncrement);
        }
    }
}

/* ── ISR & GPIO ───────────────────────────────────────────────────────────── */

static void IRAM_ATTR imu_int1_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    if (g_imu_task_handle)
        xTaskNotifyFromISR(g_imu_task_handle, 1u, eIncrement, &woken);
    portYIELD_FROM_ISR(woken);
}

static esp_err_t int1_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << IMU_INT1_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    return gpio_isr_handler_add(IMU_INT1_GPIO, imu_int1_isr, NULL);
}

/* ── IMU Acquisition Task ─────────────────────────────────────────────────── */

static void imu_task(void *arg)
{
    while (true) {
        if (!imu_fifo_work_pending()) {
            const uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            g_stats.fifo_interrupts += n;
        }
        do {
            imu_handle_event_sources();
            imu_drain_fifo();
        } while (imu_fifo_work_pending());
    }
}

/* ── Low-rate Debug / Telemetry Task ─────────────────────────────────────── */

static void imu_debug_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(LIVE_PRINT_PERIOD_MS));
    }
}

/* ── App Entry Point ──────────────────────────────────────────────────────── */

void app_main(void)
{
    i2c_init();

    if (detect_lsm6d_address() != ESP_OK) return;

    if (imu_configure() != ESP_OK) return;

    if (xTaskCreate(imu_task, "imu_acq", IMU_TASK_STACK_WORDS, NULL,
                    IMU_TASK_PRIORITY, &g_imu_task_handle) != pdPASS) return;

    if (xTaskCreate(imu_debug_task, "imu_dbg", IMU_DEBUG_TASK_STACK_WORDS, NULL,
                    IMU_DEBUG_TASK_PRIORITY, NULL) != pdPASS) return;

    if (int1_gpio_init() != ESP_OK) return;

    if (gpio_get_level(IMU_INT1_GPIO))
        xTaskNotify(g_imu_task_handle, 1u, eIncrement);
}