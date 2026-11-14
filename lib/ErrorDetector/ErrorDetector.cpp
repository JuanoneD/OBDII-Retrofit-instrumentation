#include "ErrorDetector.h"
#include "VehicleData.h"
#include "DebugSerial.h"

// =============================================================================
// Thresholds / timings (milliseconds)
// =============================================================================
static constexpr unsigned long MS_PER_MINUTE = 60000UL;

// 1  Alta temperatura
static constexpr int           TEMP_HIGH_C          = 105;
static constexpr unsigned long TEMP_HIGH_DWELL      = 3000;

// 2  Temperatura crítica
static constexpr int           TEMP_CRITICAL_C      = 115;
static constexpr unsigned long TEMP_CRITICAL_DWELL  = 1000;

// 3  Motor frio
static constexpr int           TEMP_COLD_C          = 70;
static constexpr unsigned long COLD_ENGINE_TIME     = 10 * MS_PER_MINUTE;

// 4  Aquecimento lento
static constexpr unsigned long SLOW_WARMUP_TIME     = 5 * MS_PER_MINUTE;
static constexpr int           SLOW_WARMUP_RISE_C   = 5;   // < X °C of rise in the window

// 5  Rotação alta em baixa velocidade
static constexpr int           HIGH_RPM             = 4000;
static constexpr int           LOW_SPEED_KMH        = 20;
static constexpr unsigned long HIGH_RPM_LOW_SPD_DWELL = 3000;

// 6  Marcha lenta instável
static constexpr int           IDLE_OSCILLATION_RPM = 300; // +-150
static constexpr unsigned long IDLE_OSC_DWELL       = 2000; // sustained via window spread

// 7  Marcha lenta alta (com motor quente)
static constexpr int           HIGH_IDLE_RPM        = 1100;
static constexpr int           WARM_ENGINE_C        = 80;
static constexpr unsigned long HIGH_IDLE_DWELL      = 3000;

// 8  Marcha lenta baixa / motor morrendo
static constexpr int           LOW_IDLE_RPM         = 550;
static constexpr unsigned long LOW_IDLE_DWELL       = 2000;

// 9  Carga do motor alta
static constexpr float         HIGH_LOAD_PCT        = 90.0f;
static constexpr unsigned long HIGH_LOAD_DWELL      = 10000;

// 10 Carga alta em marcha lenta
static constexpr float         HIGH_LOAD_IDLE_PCT   = 50.0f;
static constexpr unsigned long HIGH_LOAD_IDLE_DWELL = 3000;

// 11 Carga baixa em movimento
static constexpr int           MOVING_SPEED_KMH     = 60;
static constexpr float         LOW_LOAD_PCT         = 10.0f;
static constexpr unsigned long LOW_LOAD_DWELL       = 3000;

// 12/13 Mistura pobre/rica (LTFT) - "por muito tempo"
static constexpr float         LTFT_LEAN_PCT        = 15.0f;
static constexpr float         LTFT_RICH_PCT        = -15.0f;
static constexpr unsigned long LTFT_DWELL           = 60000;

// 14 Fuel trim no limite
static constexpr float         LTFT_LIMIT_PCT       = 25.0f;
static constexpr unsigned long LTFT_LIMIT_DWELL     = 10000;

// 15 Superaquecimento em marcha lenta
static constexpr int           OVERHEAT_IDLE_C      = 100;
static constexpr unsigned long OVERHEAT_IDLE_DWELL  = 10000;

// 16 Superaquecimento com carga alta
static constexpr int           OVERHEAT_C           = 100;
static constexpr float         OVERHEAT_LOAD_PCT    = 80.0f;
static constexpr unsigned long OVERHEAT_LOAD_DWELL  = 5000;

// Default cooldown between firings of the same error
static constexpr unsigned long DEFAULT_COOLDOWN     = 5 * MS_PER_MINUTE;
// One-shot-per-trip errors (cold engine / slow warmup)
static constexpr unsigned long ONESHOT_COOLDOWN     = 30 * MS_PER_MINUTE;

// =============================================================================
// Titles / descriptions (backend error table, ASCII-safe for LCD + JSON)
// =============================================================================
struct ErrorDef {
    const char* title;
    const char* description;
};

static const ErrorDef ERROR_DEFS[16] = {
    /* 1  */ { "Alta temperatura",  "Temperatura do motor passou de 105 C" },
    /* 2  */ { "Temp. critica!",    "Risco de superaquecimento/dano ao motor, parar o veiculo" },
    /* 3  */ { "Motor frio",        "Motor nao atingiu temperatura de trabalho apos 10 min (possivel termostato preso aberto)" },
    /* 4  */ { "Aquecimento lento", "Temperatura sobe muito devagar, termostato suspeito" },
    /* 5  */ { "Rotacao alta",      "RPM > 4000 com velocidade < 20 km/h: possivel embreagem patinando ou cambio com problema" },
    /* 6  */ { "Lenta instavel",    "RPM oscilando em marcha lenta (corpo de borboleta sujo, vazamento de vacuo, velas)" },
    /* 7  */ { "Marcha lenta alta", "Rotacao elevada parado com motor quente (borboleta, sensor, vazamento de ar)" },
    /* 8  */ { "Lenta baixa",       "Rotacao < 550 rpm parado, risco do motor apagar" },
    /* 9  */ { "Carga alta",        "Carga do motor acima de 90% por tempo prolongado" },
    /* 10 */ { "Carga alta lenta",  "Carga incomum parado (vazamento de vacuo, falha de ignicao)" },
    /* 11 */ { "Carga baixa mov.",  "Carga < 10% a mais de 60 km/h: possivel sensor com defeito (MAF/MAP)" },
    /* 12 */ { "Mistura pobre",     "LTFT > +15% por muito tempo: vazamento de vacuo, MAF sujo, bico entupido, bomba fraca" },
    /* 13 */ { "Mistura rica",      "LTFT < -15% por muito tempo: bico vazando, sonda lambda, filtro de ar sujo" },
    /* 14 */ { "Fuel trim limite",  "Correcao proxima do limite da ECU, falha iminente do sistema de combustivel" },
    /* 15 */ { "Superaq. na lenta", "Temperatura > 100 C parado com motor ligado: possivel falha de ventoinha" },
    /* 16 */ { "Superaq. c/ carga", "Temperatura > 100 C junto com carga > 80%" },
};

// =============================================================================
// Static state
// =============================================================================
int ErrorDetector::newErrorDetectedFlag = 0;
char ErrorDetector::m_activeTitle[21] = "";
unsigned long ErrorDetector::m_activeTitleMs = 0;
unsigned long ErrorDetector::m_lastUpdateMs = 0;

// How long the LCD keeps showing the last error title (row 3 overlay).
static constexpr unsigned long ERROR_TITLE_DISPLAY_MS = 15000;

const char* ErrorDetector::getActiveErrorTitle() {
    if (m_activeTitleMs != 0 && millis() - m_activeTitleMs < ERROR_TITLE_DISPLAY_MS) {
        return m_activeTitle;
    }
    return nullptr;
}

ErrorDetector::ConditionState ErrorDetector::m_states[16] = {};
unsigned long ErrorDetector::m_engineOnMs = 0;
unsigned long ErrorDetector::m_continuousRunStartMs = 0;
int ErrorDetector::m_tempAtRunStart = 0;
int ErrorDetector::m_idleRpmWindow[IDLE_WINDOW_SAMPLES] = {};
int ErrorDetector::m_idleWindowIndex = 0;
bool ErrorDetector::m_idleWindowFull = false;

// =============================================================================
// Core
// =============================================================================

void ErrorDetector::checkCondition(ErrorId id, bool conditionActive,
                                   unsigned long dwellMs, unsigned long cooldownMs,
                                   const String& title, const String& description) {
    ConditionState& st = m_states[(int)id - 1];
    unsigned long now = millis();

    if (!conditionActive) {
        st.conditionStartMs = 0;
        return;
    }

    if (st.conditionStartMs == 0) {
        st.conditionStartMs = now;
        return;
    }

    if (now - st.conditionStartMs < dwellMs) return;
    if (st.lastFiredMs != 0 && now - st.lastFiredMs < cooldownMs) return;

    st.lastFiredMs = now;
    pushError(id, title, description);
}

void ErrorDetector::pushError(ErrorId id, const String& title, const String& description) {
    DebugSerial::println("[ErrorDetector] Error #" + String((int)id) + " triggered: " + title);
    VehicleData::getInstance().pushDetectedError(title, description);
    newErrorDetectedFlag = 1;

    // Stage the title for the LCD overlay (truncated to the 20-char width).
    strncpy(m_activeTitle, ERROR_DEFS[(int)id - 1].title, sizeof(m_activeTitle) - 1);
    m_activeTitle[sizeof(m_activeTitle) - 1] = '\0';
    m_activeTitleMs = millis();
}

void ErrorDetector::update() {
    VehicleData& v = VehicleData::getInstance();

    if (v.getObdiiStatus() != OBDIISTATUS::CONNECTED ||
        v.getEcuStatus() != ECUSTATUS::ONLINE) {
        // Vehicle link down: reset all dwell windows so stale data doesn't
        // fire on reconnect.
        for (int i = 0; i < 16; i++) m_states[i].conditionStartMs = 0;
        m_continuousRunStartMs = 0;
        m_lastUpdateMs = 0;
        m_idleWindowIndex = 0;
        m_idleWindowFull = false;
        return;
    }

    int   rpm   = v.getEngineRPM();
    int   speed = v.getVehicleSpeed();
    int   temp  = v.getCoolantTemp();
    float load  = v.getEngineLoad();
    float ltft  = v.getLongTermFuelTrim();
    unsigned long now = millis();

    // --- Engine running-time tracking -------------------------------------
    bool engineRunning = (rpm > 0);
    unsigned long deltaMs = (m_lastUpdateMs != 0) ? (now - m_lastUpdateMs) : 0;
    m_lastUpdateMs = now;
    if (engineRunning && deltaMs > 0 && deltaMs < 10000) {
        m_engineOnMs += deltaMs;

        if (m_continuousRunStartMs == 0) {
            m_continuousRunStartMs = now;
            m_tempAtRunStart = temp;
        }
    } else {
        m_continuousRunStartMs = 0;
        m_tempAtRunStart = 0;
    }

    // --- Idle RPM window (instability / spread) ----------------------------
    bool atIdle = (speed == 0 && engineRunning);
    if (atIdle) {
        m_idleRpmWindow[m_idleWindowIndex] = rpm;
        m_idleWindowIndex = (m_idleWindowIndex + 1) % IDLE_WINDOW_SAMPLES;
        if (m_idleWindowIndex == 0) m_idleWindowFull = true;
    } else {
        m_idleWindowIndex = 0;
        m_idleWindowFull = false;
    }

    int idleMin = rpm, idleMax = rpm;
    if (atIdle) {
        int samples = m_idleWindowFull ? IDLE_WINDOW_SAMPLES : m_idleWindowIndex;
        if (samples > 0) {
            idleMin = m_idleRpmWindow[0];
            idleMax = m_idleRpmWindow[0];
            for (int i = 1; i < samples; i++) {
                if (m_idleRpmWindow[i] < idleMin) idleMin = m_idleRpmWindow[i];
                if (m_idleRpmWindow[i] > idleMax) idleMax = m_idleRpmWindow[i];
            }
        }
    }

    // --- 1 / 2: Coolant temperature ----------------------------------------
    checkCondition(ErrorId::HIGH_TEMP, temp > TEMP_HIGH_C,
                   TEMP_HIGH_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[0].title, ERROR_DEFS[0].description);
    checkCondition(ErrorId::CRITICAL_TEMP, temp > TEMP_CRITICAL_C,
                   TEMP_CRITICAL_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[1].title, ERROR_DEFS[1].description);

    // --- 3: Motor frio (one-shot per trip) ---------------------------------
    checkCondition(ErrorId::COLD_ENGINE,
                   engineRunning && temp < TEMP_COLD_C && m_engineOnMs >= COLD_ENGINE_TIME,
                   0, ONESHOT_COOLDOWN,
                   ERROR_DEFS[2].title, ERROR_DEFS[2].description);

    // --- 4: Aquecimento lento (one-shot per trip) ---------------------------
    bool slowWarmup = engineRunning &&
                      (now - m_continuousRunStartMs) >= SLOW_WARMUP_TIME &&
                      temp < TEMP_COLD_C &&
                      (temp - m_tempAtRunStart) < SLOW_WARMUP_RISE_C;
    checkCondition(ErrorId::SLOW_WARMUP, slowWarmup,
                   0, ONESHOT_COOLDOWN,
                   ERROR_DEFS[3].title, ERROR_DEFS[3].description);

    // --- 5: Rotação alta em baixa velocidade --------------------------------
    checkCondition(ErrorId::HIGH_RPM_LOW_SPEED,
                   engineRunning && rpm > HIGH_RPM && speed < LOW_SPEED_KMH,
                   HIGH_RPM_LOW_SPD_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[4].title, ERROR_DEFS[4].description);

    // --- 6: Marcha lenta instável -------------------------------------------
    bool idleOscillating = atIdle && m_idleWindowFull &&
                           (idleMax - idleMin) > IDLE_OSCILLATION_RPM;
    checkCondition(ErrorId::UNSTABLE_IDLE, idleOscillating,
                   IDLE_OSC_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[5].title, ERROR_DEFS[5].description);

    // --- 7: Marcha lenta alta ------------------------------------------------
    checkCondition(ErrorId::HIGH_IDLE,
                   atIdle && temp > WARM_ENGINE_C && rpm > HIGH_IDLE_RPM,
                   HIGH_IDLE_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[6].title, ERROR_DEFS[6].description);

    // --- 8: Marcha lenta baixa / motor morrendo ------------------------------
    checkCondition(ErrorId::LOW_IDLE,
                   atIdle && rpm > 0 && rpm < LOW_IDLE_RPM,
                   LOW_IDLE_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[7].title, ERROR_DEFS[7].description);

    // --- 9: Carga do motor alta ----------------------------------------------
    checkCondition(ErrorId::HIGH_LOAD,
                   engineRunning && load > HIGH_LOAD_PCT,
                   HIGH_LOAD_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[8].title, ERROR_DEFS[8].description);

    // --- 10: Carga alta em marcha lenta ---------------------------------------
    checkCondition(ErrorId::HIGH_LOAD_IDLE,
                   atIdle && load > HIGH_LOAD_IDLE_PCT,
                   HIGH_LOAD_IDLE_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[9].title, ERROR_DEFS[9].description);

    // --- 11: Carga baixa em movimento -----------------------------------------
    checkCondition(ErrorId::LOW_LOAD_MOVING,
                   speed > MOVING_SPEED_KMH && load < LOW_LOAD_PCT,
                   LOW_LOAD_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[10].title, ERROR_DEFS[10].description);

    // --- 12/13/14: Long Term Fuel Trim ----------------------------------------
    checkCondition(ErrorId::LEAN_MIXTURE,
                   ltft > LTFT_LEAN_PCT,
                   LTFT_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[11].title, ERROR_DEFS[11].description);
    checkCondition(ErrorId::RICH_MIXTURE,
                   ltft < LTFT_RICH_PCT,
                   LTFT_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[12].title, ERROR_DEFS[12].description);
    checkCondition(ErrorId::FUEL_TRIM_LIMIT,
                   fabsf(ltft) > LTFT_LIMIT_PCT,
                   LTFT_LIMIT_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[13].title, ERROR_DEFS[13].description);

    // --- 15: Superaquecimento em marcha lenta ----------------------------------
    checkCondition(ErrorId::OVERHEAT_IDLE,
                   atIdle && temp > OVERHEAT_IDLE_C,
                   OVERHEAT_IDLE_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[14].title, ERROR_DEFS[14].description);

    // --- 16: Superaquecimento com carga alta -----------------------------------
    checkCondition(ErrorId::OVERHEAT_LOAD,
                   engineRunning && temp > OVERHEAT_C && load > OVERHEAT_LOAD_PCT,
                   OVERHEAT_LOAD_DWELL, DEFAULT_COOLDOWN,
                   ERROR_DEFS[15].title, ERROR_DEFS[15].description);
}
