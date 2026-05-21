/* HW_Sound.cpp - Asynchroner Sound-Executor (State-Machine)
   v0.6 PAM8403 Audio Amplifier Edition (Core 2.x Kompatibel)
   
   HEILUNG für Audio-Verstärker:
   - Ersetzt tone() durch natives LEDC PWM für Software-Volume-Control.
   - Nutzt asymmetrische PWM (Nadelimpulse), um Übersteuern des Amps zu verhindern.
   - Zieht den Pin bei Stille hart auf LOW (Verhindert Grundrauschen/Hiss am Amp).
   - Präzise Step-by-Step State Machine für sauberes Timing.
*/

#include <Arduino.h>
#include "Globals.h"
#include "Config.h"

// =============================================================================
// 1. QUEUE EINSPRUNG (Von überall aufrufbar)
// =============================================================================

void uiTriggerBeep(SoundID id) {
    if (xSoundQueue != NULL) {
        xQueueSend(xSoundQueue, &id, 0);
    }
}

// =============================================================================
// 2. LEGACY WRAPPER (Abwärtskompatibilität für alte Mode_*.cpp)
// =============================================================================

void beepNav()          { uiTriggerBeep(SND_NAV); }
void beepValue()        { uiTriggerBeep(SND_VALUE); }
void beepOk()           { uiTriggerBeep(SND_OK); }
void beepClick()        { uiTriggerBeep(SND_CLICK); }
void beepHint()         { uiTriggerBeep(SND_HINT); }
void beepWarnLong()     { uiTriggerBeep(SND_WARN); }
void beepWizardStep()   { uiTriggerBeep(SND_CAL_STEP); }
void beepWizardSave()   { uiTriggerBeep(SND_DONE); }
void beepStartPattern() { uiTriggerBeep(SND_START_PATTERN); }
void beepEndPattern()   { uiTriggerBeep(SND_END_PATTERN); }

// CODE_REVIEW FIX (K06): Fehlende Definitionen für in Globals.h deklarierte Funktionen.
// beepDone() und beepAlarm() waren deklariert aber nirgends implementiert.
// Ohne Definition würde jeder Aufruf einen Linker-Fehler verursachen.
void beepDone()         { uiTriggerBeep(SND_DONE); }
void beepAlarm()        { uiTriggerBeep(SND_ALARM); }

// PFLICHTENHEFT FIX: allowUiBeeps() war in Globals.h deklariert, aber nirgends definiert.
// Steuert, ob UI-Beeps erlaubt sind (z.B. Unterdrückung während Belichtung).
// CODE_REVIEW S03 Fix: Prüfung auf starttime != 0 (nicht == 1).
bool allowUiBeeps() {
    return (starttime == 0);
}

// =============================================================================
// 3. HARDWARE PWM SYNTHESIZER (PAM8403 spezifisch)
// =============================================================================

// ESP32 Core 2.x verlangt einen dedizierten PWM-Kanal
static const int BUZZER_CHANNEL = 0; 

static void setTone(uint32_t freq) {
    if (freq == 0) {
        // Absolute Stille: PWM abschalten und Pin hart auf GND ziehen.
        // Das verhindert statisches Rauschen (Hiss) am Verstärker-Eingang.
        ledcWrite(BUZZER_CHANNEL, 0);
        ledcDetachPin(PIN_BUZZER);
        pinMode(PIN_BUZZER, OUTPUT);
        digitalWrite(PIN_BUZZER, LOW); 
        return;
    }
    
    // Core 2.x API: Setup auf Kanal 0, Frequenz, 8-Bit Auflösung
    ledcSetup(BUZZER_CHANNEL, freq, 8);
    ledcAttachPin(PIN_BUZZER, BUZZER_CHANNEL);
    
    // ROOT CAUSE FIX: Software-Volume Control für den Audio-Amp!
    // Statt brachialer 50% Duty Cycle (127), nutzen wir winzige Nadelimpulse.
    // Das senkt den Energiepegel für den PAM8403 drastisch und erzeugt klare Töne.
    uint32_t duty = (globalSet.soundMode == SOUND_QUIET) ? 2 : 15; 
    
    ledcWrite(BUZZER_CHANNEL, duty);
}

// =============================================================================
// 4. ASYNCHRONE STATE-MACHINE (Läuft im TaskIO auf Core 0)
// =============================================================================

static bool isPlaying = false;
static unsigned long nextActionMs = 0;
static SoundID currentSound = SND_NAV;
static int currentStep = 0;

void processSoundQueue() {
    if (globalSet.soundMode == SOUND_OFF) {
        SoundID dump;
        while(xQueueReceive(xSoundQueue, &dump, 0) == pdTRUE) {}
        if (isPlaying) { setTone(0); isPlaying = false; }
        return;
    }

    unsigned long now = millis();

    // A. Neuen Sound laden
    if (!isPlaying) {
        SoundID nextId;
        if (xQueueReceive(xSoundQueue, &nextId, 0) == pdTRUE) {
            
            // UI-Beeps während der Belichtung blockieren!
            if (starttime != 0 && nextId != SND_START_PATTERN && nextId != SND_END_PATTERN && nextId != SND_CLICK) {
                return; 
            }

            isPlaying = true;
            currentSound = nextId;
            currentStep = 0;
            nextActionMs = now; 
        }
    }

    // B. Sound-Sequenzer (Exakte Kontrolle über Ton- und Pausenzeiten)
    if (isPlaying && now >= nextActionMs) {
        
        switch(currentSound) {
            case SND_NAV:
                if (currentStep == 0) { setTone(800); nextActionMs = now + 60; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            case SND_OK:
                if (currentStep == 0) { setTone(2200); nextActionMs = now + 60; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            case SND_VALUE:
                if (currentStep == 0) { setTone(900); nextActionMs = now + 65; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            case SND_CLICK:
                // Metronom Klick: Sehr kurz, sehr prägnant
                if (currentStep == 0) { setTone(2600); nextActionMs = now + 18; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            case SND_BACK:
                // H07 FIX: Absteigender Doppelton für BACK/Abbruch
                if (currentStep == 0) { setTone(1200); nextActionMs = now + 50; currentStep++; }
                else if (currentStep == 1) { setTone(0); nextActionMs = now + 30; currentStep++; }
                else if (currentStep == 2) { setTone(800); nextActionMs = now + 80; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            case SND_WARN:
                if (currentStep == 0) { setTone(400); nextActionMs = now + 320; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            case SND_HINT:
                if (currentStep == 0) { setTone(500); nextActionMs = now + 140; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;

            case SND_LIMIT:
                if (currentStep == 0) { setTone(350); nextActionMs = now + 60; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;

            case SND_PHASE_READY:
                if (currentStep == 0) { setTone(2500); nextActionMs = now + 40; currentStep++; }
                else if (currentStep == 1) { setTone(0); nextActionMs = now + 40; currentStep++; }
                else if (currentStep == 2) { setTone(2500); nextActionMs = now + 40; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;

            case SND_AUTO_PROP:
                if (currentStep == 0) { setTone(1200); nextActionMs = now + 50; currentStep++; }
                else if (currentStep == 1) { setTone(0); nextActionMs = now + 25; currentStep++; }
                else if (currentStep == 2) { setTone(1500); nextActionMs = now + 50; currentStep++; }
                else if (currentStep == 3) { setTone(0); nextActionMs = now + 25; currentStep++; }
                else if (currentStep == 4) { setTone(1800); nextActionMs = now + 50; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;

            case SND_CAL_STEP:
                if (currentStep == 0) { setTone(3000); nextActionMs = now + 10; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;

            case SND_ALARM:
                if (currentStep == 0) { setTone(400); nextActionMs = now + 200; currentStep++; }
                else if (currentStep == 1) { setTone(0); nextActionMs = now + 100; currentStep++; }
                else if (currentStep == 2) { setTone(400); nextActionMs = now + 200; currentStep++; }
                else if (currentStep == 3) { setTone(0); nextActionMs = now + 100; currentStep++; }
                else if (currentStep == 4) { setTone(400); nextActionMs = now + 200; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            case SND_DONE: // z.B. beepWizardSave (Doppel-Beep)
                if (currentStep == 0) { setTone(1800); nextActionMs = now + 50; currentStep++; }
                else if (currentStep == 1) { setTone(0); nextActionMs = now + 50; currentStep++; } // Explizite Pause
                else if (currentStep == 2) { setTone(2200); nextActionMs = now + 80; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;

            case SND_START_PATTERN:
                if (currentStep == 0) { setTone(1100); nextActionMs = now + 80; currentStep++; }
                else if (currentStep == 1) { setTone(0); nextActionMs = now + 20; currentStep++; }
                else if (currentStep == 2) { setTone(1400); nextActionMs = now + 80; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            case SND_END_PATTERN:
                if (currentStep == 0) { setTone(1400); nextActionMs = now + 80; currentStep++; }
                else if (currentStep == 1) { setTone(0); nextActionMs = now + 20; currentStep++; }
                else if (currentStep == 2) { setTone(1100); nextActionMs = now + 80; currentStep++; }
                else { setTone(0); isPlaying = false; }
                break;
                
            default:
                setTone(0);
                isPlaying = false; 
                break;
        }
    }
}