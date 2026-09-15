#include "DisplayManager.h"
#include "VehicleData.h"
#include "DebugSerial.h"

// Full-block character available on HD44780 character ROMs.
static constexpr uint8_t LCD_BLOCK_CHAR = 0xFF;

// =============================================================================
// Construction / Initialization
// =============================================================================

DisplayManager::DisplayManager()
    : m_lcd(0x27, 20, 4),
      m_cols(20),
      m_rows(4),
      m_initialized(false),
      m_currentMode(DisplayMode::NONE),
      m_cacheRpmBarBlocks(-1) {}

void DisplayManager::begin(uint8_t lcd_addr, uint8_t cols, uint8_t rows) {
    // Re-instantiate the underlying driver with the requested address/geometry.
    m_lcd = LiquidCrystal_I2C(lcd_addr, cols, rows);
    m_cols = cols;
    m_rows = rows;

    m_lcd.init();
    m_lcd.backlight();
    m_lcd.clear(); // ONLY allowed clear() in the entire lifecycle.

    invalidateCache();
    m_currentMode = DisplayMode::NONE;
    m_initialized = true;

    DebugSerial::println("DisplayManager initialized (20x4 I2C).");

    // Render the appropriate screen for the current state.
    updateAll();
}

void DisplayManager::invalidateCache() {
    m_cacheRpm          = "";
    m_cacheGas          = "";
    m_cacheSpeed        = "";
    m_cacheTemp         = "";
    m_cacheLoad         = "";
    m_cacheLtft         = "";
    m_cacheRpmBarBlocks = -1;
}

// =============================================================================
// String helpers
// =============================================================================

static String padRight(const String& s, uint8_t width) {
    String out = s;
    while (out.length() < width) out += ' ';
    if (out.length() > width) out = out.substring(0, width);
    return out;
}

static String centerText(const String& s, uint8_t width) {
    if (s.length() >= width) return s.substring(0, width);
    uint8_t total = width - s.length();
    uint8_t left = total / 2;
    uint8_t right = total - left;
    String out;
    for (uint8_t i = 0; i < left; ++i) out += ' ';
    out += s;
    for (uint8_t i = 0; i < right; ++i) out += ' ';
    return out;
}

// =============================================================================
// Formatting helpers
// =============================================================================

String DisplayManager::formatRpm(int rpm) {
    if (rpm < 0) rpm = 0;
    if (rpm > 9999) rpm = 9999;
    char buf[8];
    snprintf(buf, sizeof(buf), "%4d", rpm);
    return String(buf);
}

String DisplayManager::formatGas(float liters) {
    if (liters < 0.0f)   liters = 0.0f;
    if (liters > 99.9f)  liters = 99.9f;
    char buf[8];
    snprintf(buf, sizeof(buf), "%4.1f", liters);
    return String(buf);
}

String DisplayManager::formatSpeed(int kmh) {
    if (kmh < 0)   kmh = 0;
    if (kmh > 999) kmh = 999;
    char buf[8];
    snprintf(buf, sizeof(buf), "%3d", kmh);
    return String(buf);
}

String DisplayManager::formatTemp(int celsius) {
    if (celsius < -9) celsius = -9;
    if (celsius > 199) celsius = 199;
    char buf[8];
    snprintf(buf, sizeof(buf), "%2d", celsius);
    return String(buf);
}

String DisplayManager::formatLoad(float loadPct) {
    if (loadPct < 0.0f)   loadPct = 0.0f;
    if (loadPct > 100.0f) loadPct = 100.0f;
    char buf[8];
    snprintf(buf, sizeof(buf), "%3d", (int)(loadPct + 0.5f));
    return String(buf);
}

String DisplayManager::formatLtft(float trimPct) {
    if (trimPct < -99.9f) trimPct = -99.9f;
    if (trimPct > 99.9f)  trimPct = 99.9f;
    char buf[8];
    snprintf(buf, sizeof(buf), "%4.1f", trimPct); // width 4 (e.g. "-2.3", " 3.1")
    return String(buf);
}

// =============================================================================
// Cache-aware writers
// =============================================================================

void DisplayManager::writeIfChanged(uint8_t col, uint8_t row,
                                    const String& value, String& cache) {
    if (value == cache) {
        return; // No I2C traffic when nothing changed.
    }
    m_lcd.setCursor(col, row);
    m_lcd.print(value);
    cache = value;
}

void DisplayManager::writeCenteredRow(uint8_t row, const String& text) {
    m_lcd.setCursor(0, row);
    m_lcd.print(centerText(text, m_cols));
}

// =============================================================================
// Mode handling
// =============================================================================

void DisplayManager::enterMode(DisplayMode mode) {
    if (m_currentMode == mode) return;
    m_currentMode = mode;
    invalidateCache();

    // Wipe screen without clear(): overwrite every row with spaces.
    String blank = padRight("", m_cols);
    for (uint8_t r = 0; r < m_rows; ++r) {
        m_lcd.setCursor(0, r);
        m_lcd.print(blank);
    }

    switch (mode) {
        case DisplayMode::STATUS_CONNECTING:
            DebugSerial::println("Display: Connecting OBDII...");
            break;
        case DisplayMode::STATUS_WAIT_ECU:
            DebugSerial::println("Display: Waiting for ECU...");
            break;
        case DisplayMode::DASHBOARD:
            DebugSerial::println("Display: Dashboard active.");
            break;
        default:
            break;
    }
}

// =============================================================================
// Renderers
// =============================================================================

void DisplayManager::renderStatusConnecting() {
    writeCenteredRow(1, "Connecting OBDII...");
    writeCenteredRow(2, "");
}

void DisplayManager::renderStatusWaitEcu() {
    writeCenteredRow(1, "OBDII Connected");
    writeCenteredRow(2, "Waiting for ECU...");
}

void DisplayManager::renderDashboard() {
    VehicleData& v = VehicleData::getInstance();

    // Row 0: "RPM:####  GAS:##.#L " (20 cols)
    //         0123456789012345678901
    if (m_cacheRpm.length() == 0) {
        m_lcd.setCursor(0, 0);
        m_lcd.print("RPM:");
        m_lcd.setCursor(10, 0);
        m_lcd.print("GAS:");
        m_lcd.setCursor(19, 0);
        m_lcd.print("L");
    }
    writeIfChanged(4,  0, formatRpm(v.getEngineRPM()),     m_cacheRpm);
    writeIfChanged(14, 0, formatGas(v.getGasolineLevel()), m_cacheGas);

    // Row 1: "VEL:### km/h TEMP:##C"  -> that's 21 chars, trim to 20 by
    // dropping the space between km/h and TEMP:
    // "VEL:### km/hTEMP:##C" -> 20 chars.
    if (m_cacheSpeed.length() == 0) {
        m_lcd.setCursor(0, 1);
        m_lcd.print("VEL:");
        m_lcd.setCursor(7, 1);
        m_lcd.print(" km/h");
        m_lcd.setCursor(12, 1);
        m_lcd.print("TEMP:");
        m_lcd.setCursor(19, 1);
        m_lcd.print("C");
    }
    writeIfChanged(4,  1, formatSpeed(v.getVehicleSpeed()), m_cacheSpeed);
    writeIfChanged(17, 1, formatTemp(v.getCoolantTemp()),   m_cacheTemp);

    // Row 2: "LOAD:###%  LTFT:##.#%" -> 21 chars; drop trailing '%'
    // Final: "LOAD:###%  LTFT:##.#" (20 chars).
    if (m_cacheLoad.length() == 0) {
        m_lcd.setCursor(0, 2);
        m_lcd.print("LOAD:");
        m_lcd.setCursor(8, 2);
        m_lcd.print("%");
        m_lcd.setCursor(11, 2);
        m_lcd.print("LTFT:");
    }
    writeIfChanged(5,  2, formatLoad(v.getEngineLoad()),        m_cacheLoad);
    writeIfChanged(16, 2, formatLtft(v.getLongTermFuelTrim()),  m_cacheLtft);

    // Row 3: RPM bar
    renderRpmBar(v.getEngineRPM());
}

void DisplayManager::renderRpmBar(int rpm) {
    if (rpm < 0) rpm = 0;
    if (rpm > RPM_BAR_MAX) rpm = RPM_BAR_MAX;

    // Linear scale RPM -> number of filled blocks (0..m_cols).
    int blocks = (int)(((long)rpm * m_cols + RPM_BAR_MAX / 2) / RPM_BAR_MAX);
    if (blocks < 0) blocks = 0;
    if (blocks > (int)m_cols) blocks = m_cols;

    if (blocks == m_cacheRpmBarBlocks) return; // cache hit, no I2C traffic.

    m_lcd.setCursor(0, 3);
    for (int i = 0; i < blocks; ++i) {
        m_lcd.write(LCD_BLOCK_CHAR);
    }
    for (int i = blocks; i < (int)m_cols; ++i) {
        m_lcd.write(' ');
    }
    m_cacheRpmBarBlocks = blocks;
}

// =============================================================================
// Public entry points
// =============================================================================

void DisplayManager::updateAll() {
    if (!m_initialized) return;

    VehicleData& v = VehicleData::getInstance();
    OBDIISTATUS obd = v.getObdiiStatus();
    ECUSTATUS   ecu = v.getEcuStatus();

    // Pick the correct mode for the current state.
    DisplayMode target;
    if (obd != OBDIISTATUS::CONNECTED) {
        target = DisplayMode::STATUS_CONNECTING;
    } else if (ecu != ECUSTATUS::ONLINE) {
        target = DisplayMode::STATUS_WAIT_ECU;
    } else {
        target = DisplayMode::DASHBOARD;
    }

    bool modeChanged = (target != m_currentMode);
    if (modeChanged) {
        enterMode(target);
    }

    switch (target) {
        case DisplayMode::STATUS_CONNECTING:
            if (modeChanged) renderStatusConnecting();
            break;
        case DisplayMode::STATUS_WAIT_ECU:
            if (modeChanged) renderStatusWaitEcu();
            break;
        case DisplayMode::DASHBOARD:
            renderDashboard();
            break;
        default:
            break;
    }
}

void DisplayManager::updateField(DisplayField field) {
    if (!m_initialized) return;
    if (m_currentMode != DisplayMode::DASHBOARD) return;

    VehicleData& v = VehicleData::getInstance();

    switch (field) {
        case DisplayField::RPM:
            writeIfChanged(4, 0, formatRpm(v.getEngineRPM()), m_cacheRpm);
            break;
        case DisplayField::GAS:
            writeIfChanged(14, 0, formatGas(v.getGasolineLevel()), m_cacheGas);
            break;
        case DisplayField::SPEED:
            writeIfChanged(4, 1, formatSpeed(v.getVehicleSpeed()), m_cacheSpeed);
            break;
        case DisplayField::TEMP:
            writeIfChanged(17, 1, formatTemp(v.getCoolantTemp()), m_cacheTemp);
            break;
        case DisplayField::LOAD:
            writeIfChanged(5, 2, formatLoad(v.getEngineLoad()), m_cacheLoad);
            break;
        case DisplayField::LTFT:
            writeIfChanged(16, 2, formatLtft(v.getLongTermFuelTrim()), m_cacheLtft);
            break;
        case DisplayField::RPM_BAR:
            renderRpmBar(v.getEngineRPM());
            break;
    }
}
