/* =============================================================================
 * IAppMode.h - DUKATIMER BETA (v0.908)
 * * Abstrakte Basisklasse (Interface) für alle App-Modi.
 * * Definiert den Lebenszyklus einer App innerhalb des AppManagers.
 * ========================================================================== */

#pragma once

#include <Arduino.h>

/**
 * Interface-Klasse für Dukatimer-Applikationen.
 * Jede App (BWDose, Setup etc.) implementiert diese Methoden.
 */
class IAppMode
{
public:
    virtual ~IAppMode() = default;

    /**
     * Wird beim Aktivieren des Modus einmalig aufgerufen.
     * Initialisiert lokalen Status und zieht Snapshots aus dem Kontext.
     */
    virtual void onEnter() = 0;

    /**
     * Verarbeitung von Eingabe-Events (Buttons, Encoder).
     * @param event Die Event-ID gemäß InputManager Mapping.
     */
    virtual void handleInput(int event) = 0;

    /**
     * Regelmäßige Aktualisierung (Core 0 Loop).
     * Hier werden UI-Daten in den AppSharedState geschrieben.
     */
    virtual void onUpdate() = 0;

    /**
     * Wird beim Verlassen des Modus einmalig aufgerufen.
     * Dient zum Aufräumen von Ressourcen oder Abbrechen von Vorgängen.
     */
    virtual void onExit() = 0;

    /**
     * @return Menschlich lesbarer Name des Modus (für Debugging).
     */
    virtual const char *getName() const = 0;
};