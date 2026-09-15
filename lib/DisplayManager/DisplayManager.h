#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
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
 *
 * DASHBOARD layout (20x4):
 *   Row 0: "RPM:XXXX  GAS:XX.XL "
 *   Row 1: "VEL:XXX km/h TEMP:XXC"
 *   Row 2: "LOAD:XXX%  LTFT:XX.X "
 *   Row 3: RPM bar (0..20 full-block chars, scales 0..RPM_BAR_MAX)
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
    // Upper bound of the RPM bar (RPM value that fills all 20 blocks).
    // Tune empirically for the target engine.
    static constexpr int RPM_BAR_MAX = 7000;

    DisplayManager();

    // Initializes the I2C LCD. This is the ONLY place lcd.clear() runs.
    void begin(uint8_t lcd_addr = 0x27, uint8_t cols = 20, uint8_t rows = 4);

    // Main entry point: inspects VehicleData status and renders the
    // appropriate screen (status message or dashboard). Cache-aware:
    // fields whose formatted value did not change are skipped.
    void updateAll();

    // Update a single dashboard field on demand (only meaningful while
    // in DASHBOARD mode; no-op otherwise).
    void updateField(DisplayField field);

    // Forces the next render to redraw everything regardless of cache.
    void invalidateCache();

private:
    LiquidCrystal_I2C m_lcd;
    uint8_t m_cols;
    uint8_t m_rows;
    bool m_initialized;
    DisplayMode m_currentMode;

    // Dashboard field cache (empty string => never written / dirty).
    String m_cacheRpm;
    String m_cacheGas;
    String m_cacheSpeed;
    String m_cacheTemp;
    String m_cacheLoad;
    String m_cacheLtft;
    int    m_cacheRpmBarBlocks; // -1 => dirty

    // Renderers per mode.
    void renderStatusConnecting();
    void renderStatusWaitEcu();
    void renderDashboard();
    void renderRpmBar(int rpm);

    // Formatting helpers – fixed-width strings so overwrites always
    // cover the previous content without needing clear().
    static String formatRpm(int rpm);
    static String formatGas(float liters);
    static String formatSpeed(int kmh);
    static String formatTemp(int celsius);
    static String formatLoad(float loadPct);
    static String formatLtft(float trimPct);

    // Cache-aware writer.
    void writeIfChanged(uint8_t col, uint8_t row,
                        const String& value, String& cache);

    // Draws a centered fixed-width message on the given row.
    void writeCenteredRow(uint8_t row, const String& text);

    // Switches to a new mode, wiping the screen (without clear()) and
    // invalidating caches so the next writes redraw everything.
    void enterMode(DisplayMode mode);
};

#endif // DISPLAY_MANAGER_H
