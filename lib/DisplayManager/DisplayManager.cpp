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

bool DisplayManager::probeI2C(uint8_t addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    return (err == 0); // 0 = ACK receive
}

bool DisplayManager::begin(uint8_t lcd_addr, uint8_t cols, uint8_t rows,
                            int8_t sda, int8_t scl) {
    m_addr = lcd_addr;
    m_cols = cols;
    m_rows = rows;

    if (sda >= 0 && scl >= 0) {
        Wire.begin(sda, scl);
    } else {
        Wire.begin();
    }
    delay(200);

    if (!probeI2C(lcd_addr)) {
        DebugSerial::println("DisplayManager: no LCD answer");
        m_initialized = false;
        return false;
    }

    m_lcd = LiquidCrystal_I2C(lcd_addr, cols, rows);
    m_lcd.init();
    m_lcd.backlight();
    m_lcd.clear(); // ONLY allowed clear() in the entire lifecycle.

    invalidateCache();
    m_currentMode = DisplayMode::NONE;
    m_initialized = true;

    DebugSerial::println("DisplayManager initialized (20x4 I2C).");

    updateAll();
    return true;
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

String DisplayManager::formatGas(float gasPct) {
    if (gasPct < 0.0f)   gasPct = 0.0f;
    if (gasPct > 100.0f) gasPct = 100.0f;
    char buf[8];
    snprintf(buf, sizeof(buf), "%3d", (int)(gasPct + 0.5f));
    return String(buf);
}

// Format speed to always occupy 3 positions (e.g. "  5", " 45", "120") to prevent residual digits
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
    snprintf(buf, sizeof(buf), "%3d", celsius);
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
    if (trimPct < -99.0f) trimPct = -99.0f;
    if (trimPct > 99.0f)  trimPct = 99.0f;
    
    // Arredonda corretamente o float para inteiro (ex: 3.2 -> 3, -1.2 -> -1)
    int val = (int)(trimPct + (trimPct >= 0 ? 0.5f : -0.5f));
    
    char buf[8];
    // Formata explicitamente com o sinal (+ ou -) seguido do número e do '%'
    snprintf(buf, sizeof(buf), "%+d%% ", val);
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

    // Row 0: "RPM: 0000        99%" (20 cols total)
    // RPM starts at col 5 (4 chars width). Gas percentage right-aligned at col 16 (3 chars width) + "%" at col 19.
    if (m_cacheRpm.length() == 0) {
        m_lcd.setCursor(0, 0);
        m_lcd.print("RPM:     "); // fixed static labels
        m_lcd.setCursor(19, 0);
        m_lcd.print("%");
    }
    float cap = v.getTankCapacity();
    float gasPct = (cap > 0.0f) ? (v.getGasolineLevel() / cap * 100.0f) : 0.0f;
    writeIfChanged(5,  0, formatRpm(v.getEngineRPM()), m_cacheRpm);
    writeIfChanged(16, 0, formatGas(gasPct),           m_cacheGas);

    // Row 1: "00 km/h            00 Cº" (20 cols total)
    // Speed right-aligned to finish cleanly at km/h, Temp right-aligned at the end with degree symbol.
    if (m_cacheSpeed.length() == 0) {
        m_lcd.setCursor(0, 1);
        m_lcd.print("    km/h");
        m_lcd.setCursor(14, 1);
        m_lcd.print("  "); // spacing
        m_lcd.setCursor(18, 1);
        m_lcd.print("C\xDF"); // \xDF prints degree symbol '°' on HD44780 ROM
    }
    // Speed uses 3 chars width (cols 0, 1, 2) ensuring single digits like '9' are cleanly padded with spaces ("  9")
    writeIfChanged(0,  1, formatSpeed(v.getVehicleSpeed()), m_cacheSpeed);
    writeIfChanged(15, 1, formatTemp(v.getCoolantTemp()),   m_cacheTemp);

    // Row 2: "LOAD: 99%    LTFT: +0%" (20 cols total)
    if (m_cacheLoad.length() == 0) {
        m_lcd.setCursor(0, 2);
        m_lcd.print("LOAD:   % LTFT: ");
    }
    writeIfChanged(5,  2, formatLoad(v.getEngineLoad()),        m_cacheLoad);
    writeIfChanged(16, 2, formatLtft(v.getLongTermFuelTrim()),  m_cacheLtft);

    // Row 3: RPM bar (intocada)
    renderRpmBar(v.getEngineRPM());
}

void DisplayManager::renderRpmBar(int rpm) {
    if (rpm < 0) rpm = 0;
    if (rpm > RPM_BAR_MAX) rpm = RPM_BAR_MAX;

    int blocks = (int)(((long)rpm * m_cols + RPM_BAR_MAX / 2) / RPM_BAR_MAX);
    if (blocks < 0) blocks = 0;
    if (blocks > (int)m_cols) blocks = m_cols;

    if (blocks == m_cacheRpmBarBlocks) return;

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
            writeIfChanged(5, 0, formatRpm(v.getEngineRPM()), m_cacheRpm);
            break;
        case DisplayField::GAS: {
            float cap = v.getTankCapacity();
            float gasPct = (cap > 0.0f) ? (v.getGasolineLevel() / cap * 100.0f) : 0.0f;
            writeIfChanged(16, 0, formatGas(gasPct), m_cacheGas);
            break;
        }
        case DisplayField::SPEED:
            writeIfChanged(0, 1, formatSpeed(v.getVehicleSpeed()), m_cacheSpeed);
            break;
        case DisplayField::TEMP:
            writeIfChanged(15, 1, formatTemp(v.getCoolantTemp()), m_cacheTemp);
            break;
        case DisplayField::LOAD:
            writeIfChanged(6, 2, formatLoad(v.getEngineLoad()), m_cacheLoad);
            break;
        case DisplayField::LTFT:
            writeIfChanged(16, 2, formatLtft(v.getLongTermFuelTrim()), m_cacheLtft);
            break;
        case DisplayField::RPM_BAR:
            renderRpmBar(v.getEngineRPM());
            break;
    }
}