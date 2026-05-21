#include "InputManager.h"
#include "HardwareManager.h"

/* =============================================================================
 * InputManager.cpp - DUKATIMER BETA (v0.916.12)
 * * REVISION: "v0.3 Muskelgedächtnis & Präzision"
 * * FIX: Encoder-Skalierung auf 1/2 Steps (1 physikalischer Klick = 1 Event).
 * * FIX: Mapping der Encoder und Taster exakt nach Pflichtenheft v0.3:
 * - Encoder 1 (Links):  Zieldosis (Zeit/EV)
 * - Encoder 2 (Mitte):  Feintuning (+/-) & ENTER
 * - Encoder 3 (Rechts): Spektral-Verhältnis (Grade) & BACK/ABORT
 * ========================================================================== */

InputManager::InputManager(AppManager *appMgr, HardwareManager *hwMgr)
    : _appMgr(appMgr),
      _hwMgr(hwMgr),
      _encSoft(),
      _encHard(),
      _encGrade(),
      _encMode(),
      // Taster (Momentary): Mapping der Pins auf interne Indizes
      _buttonPins{PIN_ENC_SOFT_SW, PIN_ENC_HARD_SW, PIN_ENC_GRADE_SW, PIN_ENC_MODE_SW, PIN_BTN_START},
      // Kippschalter (Latching): Hardware-Schalter für Licht/Raum
      _switchPins{PIN_BTN_RED, PIN_BTN_WHITE, PIN_BTN_ROOM},
      _stateRed(false),
      _stateWhite(false),
      _initialized(false)
{
    // Arrays sicher initialisieren
    for (uint8_t i = 0; i < kEncoderCount; i++)
    {
        _encoders[i] = nullptr;
        _lastCount[i] = 0;
    }
    for (uint8_t i = 0; i < 5; i++)
    {
        _btnRawState[i] = HIGH;
        _btnStableState[i] = HIGH;
        _btnLastChangeMs[i] = 0;
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        _swRawState[i] = HIGH;
        _swStableState[i] = HIGH;
        _swLastChangeMs[i] = 0;
    }
}

void InputManager::init()
{
    // PCNT (Pulse Counter) des ESP32 für Hardware-Decoding nutzen
    ESP32Encoder::useInternalWeakPullResistors = UP;

    // Initialisierung im Half-Quad Modus (2 Flanken pro vollem Zyklus)
    _encSoft.attachHalfQuad(PIN_ENC_SOFT_A, PIN_ENC_SOFT_B);    // Enc 1
    _encHard.attachHalfQuad(PIN_ENC_HARD_A, PIN_ENC_HARD_B);    // Enc 2
    _encGrade.attachHalfQuad(PIN_ENC_GRADE_A, PIN_ENC_GRADE_B); // Enc 3
    _encMode.attachHalfQuad(PIN_ENC_MODE_A, PIN_ENC_MODE_B);    // Enc 4

    _encoders[0] = &_encSoft;
    _encoders[1] = &_encHard;
    _encoders[2] = &_encGrade;
    _encoders[3] = &_encMode;

    for (uint8_t i = 0; i < kEncoderCount; ++i)
    {
        _encoders[i]->setCount(0);
        _lastCount[i] = 0;
    }

    const uint32_t now = millis();
    // Taster-Pins konfigurieren (Pullup aktiv)
    for (uint8_t i = 0; i < 5; ++i)
    {
        pinMode(_buttonPins[i], INPUT_PULLUP);
        uint8_t s = digitalRead(_buttonPins[i]);
        _btnRawState[i] = s;
        _btnStableState[i] = s;
        _btnLastChangeMs[i] = now;
    }

    // Kippschalter initialisieren und aktuellen Status an HardwareManager melden
    for (uint8_t i = 0; i < 3; ++i)
    {
        pinMode(_switchPins[i], INPUT_PULLUP);
        uint8_t s = digitalRead(_switchPins[i]);
        _swRawState[i] = s;
        _swStableState[i] = s;
        _swLastChangeMs[i] = now;
    }
    if (_hwMgr)
    {
        _stateRed = (_swStableState[0] == LOW);
        _stateWhite = (_swStableState[1] == LOW);
        _hwMgr->setSafelight(_stateRed);
        _hwMgr->setFocuslight(_stateWhite);
        _hwMgr->setRoomlight(_swStableState[2] == LOW);
        _hwMgr->syncPhysicalLights();
    }
    _initialized = true;
}

void InputManager::process()
{
    if (!_initialized)
        return;
    processEncoders();
    processButtons();
    processSwitches();
}

bool InputManager::dispatchEvent(InputEvent event)
{
    if (_appMgr)
    {
        // Not-Aus (ABORT) hat Vorrang in der Event-Queue
        bool toFront = (event == EV_ABORT);
        return _appMgr->postEvent(static_cast<int>(event), toFront);
    }
    return false;
}

void InputManager::processEncoders()
{
    for (uint8_t i = 0; i < kEncoderCount; ++i)
    {
        int64_t curCount = _encoders[i]->getCount();
        int64_t rawDiff = curCount - _lastCount[i];

        // --- SKALIERUNG AUF 1/2 STEPS ---
        // Die meisten mechanischen Encoder liefern 2 Ticks pro physikalischem Rastpunkt (Click).
        // Um ein "Überspringen" von Werten im UI zu verhindern, skalieren wir hier hart.
        int64_t diff = rawDiff / 2; // v0.3: 1 Detent = 1 Event

        if (diff == 0)
            continue;

        bool isPositive = (diff > 0);
        InputEvent ev = mapEncoderEvent(i, isPositive);

        // Begrenzung der Events pro Zyklus, um die App-Queue nicht zu fluten (Encoder-Spam Schutz)
        int64_t steps = isPositive ? diff : -diff;
        if (steps > kMaxEncoderEventsPerCycle)
            steps = kMaxEncoderEventsPerCycle;

        int64_t successfulSteps = 0;
        for (int64_t j = 0; j < steps; ++j)
        {
            if (dispatchEvent(ev))
            {
                successfulSteps++;
            }
            else
            {
                // Queue voll: Wir hören auf und behalten die restlichen Ticks für den nächsten Zyklus
                break;
            }
        }

        // WICHTIG: Wir addieren nur die erfolgreich verarbeiteten Ticks mal den Teiler zurück,
        // um keine Klicks im "Nichts" verschwinden zu lassen.
        _lastCount[i] += isPositive ? (successfulSteps * 2) : -(successfulSteps * 2);
    }
}

void InputManager::processButtons()
{
    const uint32_t now = millis();
    for (uint8_t i = 0; i < 5; ++i)
    {
        uint8_t raw = digitalRead(_buttonPins[i]);
        // Einfache Entprellung (Software Debounce)
        if (raw != _btnRawState[i])
        {
            _btnRawState[i] = raw;
            _btnLastChangeMs[i] = now;
        }
        if ((now - _btnLastChangeMs[i]) < kDebounceMs)
            continue;

        if (_btnStableState[i] == _btnRawState[i])
            continue;
        uint8_t oldStable = _btnStableState[i];
        _btnStableState[i] = _btnRawState[i];

        // Trigger bei Flankenwechsel von HIGH auf LOW (Drücken)
        if (oldStable == HIGH && _btnStableState[i] == LOW)
        {
            handleButtonState(i);
        }
    }
}

void InputManager::processSwitches()
{
    const uint32_t now = millis();
    for (uint8_t i = 0; i < 3; ++i)
    {
        uint8_t raw = digitalRead(_switchPins[i]);
        if (raw != _swRawState[i])
        {
            _swRawState[i] = raw;
            _swLastChangeMs[i] = now;
        }
        if ((now - _swLastChangeMs[i]) < kDebounceMs)
            continue;

        if (_swStableState[i] == _swRawState[i])
            continue;
        uint8_t oldStable = _swStableState[i];
        _swStableState[i] = _swRawState[i];

        // Kippschalter: Beide Flanken triggern Zustandsänderung im HardwareManager
        if (oldStable == HIGH && _swStableState[i] == LOW)
        {
            handleSwitchState(i, true);
        }
        else if (oldStable == LOW && _swStableState[i] == HIGH)
        {
            handleSwitchState(i, false);
        }
    }
}

void InputManager::handleButtonState(uint8_t idx)
{
    // Mapping der Taster auf semantische Events gemäß v0.3 Pflichtenheft
    switch (idx)
    {
    case 0:
        dispatchEvent(EV_MENU_CLICK);
        break; // Links: Menü/Optionen
    case 1:
        dispatchEvent(EV_TIME_CLICK);
        break; // Mitte: ENTER / Bestätigung
    case 2:
        dispatchEvent(EV_GRADE_CLICK);
        break; // Rechts: BACK / Abbruch
    case 3:
        dispatchEvent(EV_MODE_CLICK);
        break; // Mode-Encoder Click
    case 4:
        dispatchEvent(EV_START);
        break; // Großer Start-Taster / Hardware-Stop bei laufender Aktion
    }
}

void InputManager::handleSwitchState(uint8_t idx, bool active)
{
    if (_hwMgr)
    {
        switch (idx)
        {
        case 0:
            _stateRed = active;
            _hwMgr->setSafelight(active);
            break;
        case 1:
            _stateWhite = active;
            _hwMgr->setFocuslight(active);
            break;
        case 2:
            _hwMgr->setRoomlight(active);
            break;
        default:
            break;
        }

        _hwMgr->syncPhysicalLights();
    }
}

InputEvent InputManager::mapEncoderEvent(uint8_t encoderIndex, bool positiveDirection) const
{
    // SEMANTISCHES MAPPING (Pflichtenheft v0.3 Seite 1):
    // Index 0 (Links):  Zieldosis (Dose/Zeit) -> EV_TIME_UP/DOWN
    // Index 1 (Mitte):  Feintuning / Navi     -> EV_MENU_UP/DOWN
    // Index 2 (Rechts): Kontrast (Spectrum)   -> EV_GRADE_UP/DOWN
    // Index 3 (Mode):   App-Wechsel           -> EV_MODE_NEXT/PREV
    switch (encoderIndex)
    {
    case 0:
        return positiveDirection ? EV_TIME_UP : EV_TIME_DOWN;
    case 1:
        return positiveDirection ? EV_MENU_UP : EV_MENU_DOWN;
    case 2:
        return positiveDirection ? EV_GRADE_UP : EV_GRADE_DOWN;
    case 3:
        return positiveDirection ? EV_MODE_NEXT : EV_MODE_PREV;
    default:
        return EV_NONE;
    }
}