#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/**
 * DisplayManager
 * ------------------------------------------------------------------
 * Flicker-free 20x4 I2C LCD driver for the OBDII retrofit dashboard.
 *
 * Design rules:
 *   - lcd.clear() runs ONLY once at boot (inside begin()).
 *   - All subsequent redraws use setCursor() + overwrite with padded
 *     strings so residual characters are erased without clearing.
 *   - An internal cache holds the last value written to each field.
 *     I2C writes are skipped when the new value matches the cache.
 *
 * Display modes (chosen automatically by updateAll() based on
 * VehicleData status):
 *   - STATUS_CONNECTING : "Connecting OBDII..."
 *   - STATUS_WAIT_ECU   : "OBDII Connected" / "Waiting for ECU..."
 *   - DASHBOARD         : full telemetry + RPM bar
 */

enum class DisplayField : uint8_t {
    RPM,
    GAS,
    SPEED,
    TEMP,
    LOAD,
    LTFT,
    RPM_BAR
};

enum class DisplayMode : uint8_t {
    NONE,
    STATUS_CONNECTING,
    STATUS_WAIT_ECU,
    DASHBOARD
};

class DisplayManager {
public:
    static constexpr int RPM_BAR_MAX = 7000;

    DisplayManager();

    // Initializes I2C (with the given SDA/SCL pins) and the LCD.
    // This is the ONLY place lcd.clear() runs.
    // Returns false if the LCD did not ACK at lcd_addr (I2C wiring/address problem).
    bool begin(uint8_t lcd_addr = 0x27, uint8_t cols = 20, uint8_t rows = 4,
               int8_t sda = 21, int8_t scl = 22);

    void updateAll();
    void updateField(DisplayField field);
    void invalidateCache();

private:
    LiquidCrystal_I2C m_lcd;
    uint8_t m_cols;
    uint8_t m_rows;
    uint8_t m_addr;
    bool m_initialized;
    DisplayMode m_currentMode;

    String m_cacheRpm;
    String m_cacheGas;
    String m_cacheSpeed;
    String m_cacheTemp;
    String m_cacheLoad;
    String m_cacheLtft;
    int    m_cacheRpmBarBlocks;

    bool probeI2C(uint8_t addr);

    void renderStatusConnecting();
    void renderStatusWaitEcu();
    void renderDashboard();
    void renderRpmBar(int rpm);

    static String formatRpm(int rpm);
    static String formatGas(float gasPct);
    static String formatSpeed(int kmh);
    static String formatTemp(int celsius);
    static String formatLoad(float loadPct);
    static String formatLtft(float trimPct);

    void writeIfChanged(uint8_t col, uint8_t row,
                        const String& value, String& cache);
    void writeCenteredRow(uint8_t row, const String& text);
    void enterMode(DisplayMode mode);
};

#endif // DISPLAY_MANAGER_H