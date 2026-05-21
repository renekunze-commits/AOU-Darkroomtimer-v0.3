#include "LiveViewApp.h"

/*
 * LiveViewApp - Implementierung
 *
 * Diese Datei implementiert eine nicht-blockierende Periodik, welche die
 * Sensordaten via SystemContext anfordert und über Serial ausgibt.
 */

LiveViewApp::LiveViewApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _pm(pm), _lastMillis(0)
{
}

LiveViewApp::~LiveViewApp() {}

// Beim Betreten initialisieren wir das Zeit-Referenz-Timestamp.
void LiveViewApp::onEnter()
{
    _lastMillis = millis();
    // Hinweise: Keine Hardware-Aktionen hier ohne HardwareManager-APIs.
    Serial.printf("[LiveView] onEnter()\n");
}

// Eingaben können z.B. Tasten-Events oder UI-Buttons sein.
void LiveViewApp::handleInput(int event)
{
    // Event-Codes sind bewusst generisch gehalten. Apps sollen nicht blockieren.
    Serial.printf("[LiveView] handleInput event=%d\n", event);
}

// Nicht-blockierende Periodik: alle INTERVAL_MS ms Status abfragen und loggen.
void LiveViewApp::onUpdate()
{
    unsigned long now = millis();
    if (now - _lastMillis < INTERVAL_MS)
        return; // kein Blocking, einfach zurückkehren

    _lastMillis = now;

    // Lokale Snapshot-Variable zum Kopieren durch SystemContext
    HardwareStatus status;
    if (_ctx->getStatus(status))
    {
        // Klar kommentierte Serielle Ausgabe als Platzhalter für Display-Integration
        Serial.printf("[LiveView] HeadLux: %.2f, BaseLux: %.2f, TempAlu: %.2f, LiveDose: %.3f\n",
                      status.headLux, status.baseLux, status.tempAlu, status.liveDose);
    }
    else
    {
        Serial.printf("[LiveView] getStatus() fehlgeschlagen (Mutex/Timeout)\n");
    }
}

void LiveViewApp::onExit()
{
    Serial.printf("[LiveView] onExit()\n");
}
