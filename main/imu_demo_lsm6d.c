#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ──────────────── I2C config ──────────────── */
#define I2C_MASTER_SCL_IO       9
#define I2C_MASTER_SDA_IO       8
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000      // ← 400 kHz (fast-mode) is fine for LSM6DSV16X

/* ──────────────── LSM6DSV16X register map (from datasheet) ──────────────── */
#define LSM6D_ADDR_A    0x6A    // SDO/SA0 = GND
#define LSM6D_ADDR_B    0x6B    // SDO/SA0 = VCC

#define WHO_AM_I_REG    0x0F    // Fixed value: 0x70
#define CTRL1_REG       0x10    // Accelerometer control
#define CTRL2_REG       0x11    // Gyroscope control
#define CTRL3_REG       0x12    // BDU, IF_INC, SW_RESET
#define CTRL6_REG       0x15    // Gyroscope FS, LPF1
#define CTRL7_REG       0x16    // Analog hub / Qvar
#define CTRL8_REG       0x17    // Accelerometer FS, filters
#define STATUS_REG      0x1E
#define OUTX_L_G        0x22    // Gyroscope X low byte
#define OUTX_L_A        0x28    // Accelerometer X low byte

/* ──────────────── Sensitivity from datasheet Table 3 ──────────────── */
/*  Accelerometer FS_XL = 00 → ±2 g  → 0.061 mg/LSB  */
#define ACCEL_SENSITIVITY_MG_PER_LSB    0.061f
/*  Gyroscope FS_G = 0001 → ±250 dps → 8.75 mdps/LSB */
#define GYRO_SENSITIVITY_MDPS_PER_LSB   8.75f

/* ──────────────── STATUS_REG bit masks (Table 82) ──────────────── */
#define STATUS_XLDA     (1 << 0)    // Accelerometer data available
#define STATUS_GDA      (1 << 1)    // Gyroscope data available


/* ═══════════════════════════════════════════════════════════════════
   I2C helpers
═══════════════════════════════════════════════════════════════════ */

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static bool scan_i2c_bus(void)
{
    bool found = false;
    printf("Scanning I2C bus...\n");
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("  Found device at 0x%02X\n", addr);
            found = true;
        }
    }
    printf("Scan complete\n");
    return found;
}

static esp_err_t write_register(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, buf, 2,
                                      pdMS_TO_TICKS(1000));
}

static esp_err_t read_register(uint8_t dev_addr, uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr,
                                        &reg, 1, value, 1,
                                        pdMS_TO_TICKS(1000));
}

static esp_err_t read_burst(uint8_t dev_addr, uint8_t start_reg,
                             uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr,
                                        &start_reg, 1, buf, len,
                                        pdMS_TO_TICKS(1000));
}


/* ═══════════════════════════════════════════════════════════════════
   Sensor init and main loop
═══════════════════════════════════════════════════════════════════ */

static uint8_t detect_sensor(void)
{
    uint8_t who_am_i = 0;
    for (uint8_t addr = LSM6D_ADDR_A; addr <= LSM6D_ADDR_B; addr++) {
        esp_err_t ret = read_register(addr, WHO_AM_I_REG, &who_am_i);
        printf("WHO_AM_I @ 0x%02X: %s", addr, esp_err_to_name(ret));
        if (ret == ESP_OK) {
            printf(" = 0x%02X", who_am_i);
            if (who_am_i == 0x70) {         // datasheet: fixed value 0x70
                printf(" ✓ LSM6DSV16X detected\n");
                return addr;
            } else {
                printf(" ✗ Unexpected WHO_AM_I (expected 0x70)\n");
            }
        } else {
            printf("\n");
        }
    }
    return 0;
}

static esp_err_t sensor_init(uint8_t addr)
{
    esp_err_t ret;

    /* Step 1: Software reset (CTRL3 bit 0 = SW_RESET).
       BDU default=1, IF_INC default=1 per datasheet, SW_RESET self-clears. */
    ret = write_register(addr, CTRL3_REG, 0x01);  // SW_RESET=1
    if (ret != ESP_OK) {
        printf("SW_RESET failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for reset to complete

    /* Confirm SW_RESET has cleared */
    uint8_t ctrl3 = 0xFF;
    for (int i = 0; i < 10; i++) {
        read_register(addr, CTRL3_REG, &ctrl3);
        if (!(ctrl3 & 0x01)) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    printf("CTRL3 after reset = 0x%02X (SW_RESET %s)\n",
           ctrl3, (ctrl3 & 0x01) ? "still set!" : "cleared OK");

    /* Step 2: CTRL3 – BDU=1, IF_INC=1 (both are default=1 after reset,
       but write explicitly to be safe).
       Bit layout: BOOT|BDU|0|0|0|IF_INC|0|SW_RESET
       BDU   = bit 6 → 1  (no update until MSB+LSB both read)
       IF_INC = bit 2 → 1  (auto-increment for burst reads)
       Value = 0b01000100 = 0x44
    */
    ret = write_register(addr, CTRL3_REG, 0x44);
    if (ret != ESP_OK) { printf("CTRL3 write failed\n"); return ret; }

    /* Step 3: CTRL6 – Gyroscope full-scale
       FS_G[3:0] = 0001 → ±250 dps (8.75 mdps/LSB)
       Value = 0x01
    */
    ret = write_register(addr, CTRL6_REG, 0x01);
    if (ret != ESP_OK) { printf("CTRL6 write failed\n"); return ret; }

    /* Step 4: CTRL8 – Accelerometer full-scale
       FS_XL[1:0] = 00 → ±2 g (0.061 mg/LSB)
       Value = 0x00
    */
    ret = write_register(addr, CTRL8_REG, 0x00);
    if (ret != ESP_OK) { printf("CTRL8 write failed\n"); return ret; }

    /* Step 5: CTRL1 – Accelerometer ODR + operating mode
       Bit layout: 0|OP_MODE_XL[2:0]|ODR_XL[3:0]
       OP_MODE_XL = 000 → high-performance mode
       ODR_XL     = 0110 → 120 Hz
       Value = 0b00000110 = 0x06
    */
    ret = write_register(addr, CTRL1_REG, 0x06);
    if (ret != ESP_OK) { printf("CTRL1 write failed\n"); return ret; }

    /* Step 6: CTRL2 – Gyroscope ODR + operating mode
       Bit layout: 0|OP_MODE_G[2:0]|ODR_G[3:0]
       OP_MODE_G = 000 → high-performance mode
       ODR_G     = 0110 → 120 Hz
       Value = 0b00000110 = 0x06
    */
    ret = write_register(addr, CTRL2_REG, 0x06);
    if (ret != ESP_OK) { printf("CTRL2 write failed\n"); return ret; }

    /* Allow the ODR to settle (≥1 sample period at 120 Hz ≈ 8.3 ms) */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Readback verification */
    uint8_t rb = 0;
    printf("--- Register readback ---\n");
    read_register(addr, CTRL1_REG, &rb); printf("  CTRL1 (10h) = 0x%02X  (expect 0x06)\n", rb);
    read_register(addr, CTRL2_REG, &rb); printf("  CTRL2 (11h) = 0x%02X  (expect 0x06)\n", rb);
    read_register(addr, CTRL3_REG, &rb); printf("  CTRL3 (12h) = 0x%02X  (expect 0x44)\n", rb);
    read_register(addr, CTRL6_REG, &rb); printf("  CTRL6 (15h) = 0x%02X  (expect 0x01)\n", rb);
    read_register(addr, CTRL8_REG, &rb); printf("  CTRL8 (17h) = 0x%02X  (expect 0x00)\n", rb);
    printf("-------------------------\n");

    return ESP_OK;
}

void app_main(void)
{
    i2c_master_init();
    scan_i2c_bus();

    uint8_t active_addr = detect_sensor();
    if (active_addr == 0) {
        printf("ERROR: No valid LSM6DSV16X detected. Check wiring and SDO pin.\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (sensor_init(active_addr) != ESP_OK) {
        printf("ERROR: Sensor initialization failed.\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("\nReading IMU data (Accel: mg, Gyro: mdps)...\n");
    printf("%-12s %-8s %-8s %-8s  |  %-10s %-10s %-10s\n",
           "", "AX(mg)", "AY(mg)", "AZ(mg)", "GX(mdps)", "GY(mdps)", "GZ(mdps)");

    while (1) {
        /* Poll STATUS_REG until both XLDA (bit0) and GDA (bit1) are set */
        uint8_t status = 0;
        int timeout = 200;
        while (timeout-- > 0) {
            if (read_register(active_addr, STATUS_REG, &status) == ESP_OK &&
                (status & (STATUS_XLDA | STATUS_GDA)) == (STATUS_XLDA | STATUS_GDA)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (timeout <= 0) {
            printf("Timeout waiting for data ready (STATUS=0x%02X)\n", status);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /*
         * Burst-read gyroscope + accelerometer in one transaction:
         *   0x22: OUTX_L_G  (gyro X low)
         *   0x23: OUTX_H_G  (gyro X high)
         *   0x24: OUTY_L_G
         *   0x25: OUTY_H_G
         *   0x26: OUTZ_L_G
         *   0x27: OUTZ_H_G
         *   0x28: OUTX_L_A  (accel X low)
         *   0x29: OUTX_H_A  (accel X high)
         *   0x2A: OUTY_L_A
         *   0x2B: OUTY_H_A
         *   0x2C: OUTZ_L_A
         *   0x2D: OUTZ_H_A
         *
         * IF_INC=1 (set in CTRL3) means the register address auto-increments,
         * so a single 12-byte burst from 0x22 gives all 6 axes.
         */
        uint8_t raw[12];
        esp_err_t ret = read_burst(active_addr, OUTX_L_G, raw, sizeof(raw));
        if (ret != ESP_OK) {
            printf("Burst read failed: %s\n", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Reconstruct 16-bit two's complement values (little-endian) */
        int16_t gx_raw = (int16_t)((raw[1]  << 8) | raw[0]);
        int16_t gy_raw = (int16_t)((raw[3]  << 8) | raw[2]);
        int16_t gz_raw = (int16_t)((raw[5]  << 8) | raw[4]);
        int16_t ax_raw = (int16_t)((raw[7]  << 8) | raw[6]);
        int16_t ay_raw = (int16_t)((raw[9]  << 8) | raw[8]);
        int16_t az_raw = (int16_t)((raw[11] << 8) | raw[10]);

        /* Apply sensitivity to get physical units */
        float ax_mg   = ax_raw * ACCEL_SENSITIVITY_MG_PER_LSB;
        float ay_mg   = ay_raw * ACCEL_SENSITIVITY_MG_PER_LSB;
        float az_mg   = az_raw * ACCEL_SENSITIVITY_MG_PER_LSB;

        float gx_mdps = gx_raw * GYRO_SENSITIVITY_MDPS_PER_LSB;
        float gy_mdps = gy_raw * GYRO_SENSITIVITY_MDPS_PER_LSB;
        float gz_mdps = gz_raw * GYRO_SENSITIVITY_MDPS_PER_LSB;

        /* Convert to g and dps for readability */
        float ax_g   = ax_mg   / 1000.0f;
        float ay_g   = ay_mg   / 1000.0f;
        float az_g   = az_mg   / 1000.0f;
        float gx_dps = gx_mdps / 1000.0f;
        float gy_dps = gy_mdps / 1000.0f;
        float gz_dps = gz_mdps / 1000.0f;

        /* Convert g to m/s² */
        #define G_TO_MS2  9.80665f
        float ax_ms2 = ax_g * G_TO_MS2;
        float ay_ms2 = ay_g * G_TO_MS2;
        float az_ms2 = az_g * G_TO_MS2;

        printf("Accel[m/s2]: %7.3f %7.3f %7.3f  |  Gyro[dps]: %8.2f %8.2f %8.2f\n",
               ax_ms2, ay_ms2, az_ms2, gx_dps, gy_dps, gz_dps);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
    