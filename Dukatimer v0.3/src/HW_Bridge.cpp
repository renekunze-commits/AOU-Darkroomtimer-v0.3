#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Globals.h"

void enterNextionUploadMode() {
    // Die Bridge nutzt bewusst kleine Stack-Puffer, damit pro Schleifendurchlauf mehrere Bytes
    // am Stueck transferiert werden koennen, ohne dafuer dynamischen Speicher zu reservieren.
    uint8_t usbToNextionBuffer[128];
    uint8_t nextionToUsbBuffer[128];

    // Solange dieser Modus aktiv ist, muss Core 0 erkennen koennen, dass keinerlei regulare
    // Nextion-Kommunikation mehr stattfinden darf. Genau dafuer setzen wir explizit MODE_BRIDGE.
    currentMode = MODE_BRIDGE;

    // Defensive Absicherung: Falls der Display-Stack den Mutex wider Erwarten noch nicht
    // initialisiert hat, koennen wir keinen sicheren Bridge-Betrieb garantieren.
    if (xNexMutex == NULL) {
        /* !!! FEHLENDER KONTEXT !!! Der Nextion-Mutex wurde vor enterNextionUploadMode() nicht initialisiert. */
        smartLCD("BRIDGE ERROR", "NEX MUTEX NULL");
        currentMode = MODE_SETUP;
        return;
    }

    // Dieser Mutex-Zugriff ist die zentrale Schutzmassnahme: Wir nehmen xNexMutex ohne Timeout
    // und behalten ihn fuer die komplette Lebensdauer der Bridge. Dadurch koennen alle normalen
    // DM_sendCommand-Aufrufe das Nextion nicht mehr parallel beschreiben.
    xSemaphoreTake(xNexMutex, portMAX_DELAY);

    for (;;) {
        // Die Watchdog-Pflege bleibt auch im Bridge-Modus aktiv, weil die Schleife bewusst lang
        // laufen darf und wir keinen unnoetigen Reset durch den Task-Watchdog provozieren wollen.
        wdt_reset();

        // Die Abbruchbedingung wird in JEDEM Durchlauf zuerst geprueft. Dadurch reicht bereits
        // ein einziges Input-Event, um den Modus deterministisch und ohne Zusatzprotokoll zu
        // verlassen. Das Event wird absichtlich konsumiert und nicht weitergereicht.
        InputEvent abortEvent;
        if (xQueueReceive(xInputQueue, &abortEvent, 0) == pdTRUE) {
            break;
        }

        bool transferredAnyByte = false;

        // Richtung 1: USB vom PC -> Serial2 zum Nextion. Wir lesen so viele Bytes wie aktuell
        // sofort verfuegbar sind, maximal aber die Puffergroesse dieses Durchlaufs.
        size_t usbAvailable = (size_t)Serial.available();
        if (usbAvailable > 0) {
            size_t usbChunk = usbAvailable;
            if (usbChunk > sizeof(usbToNextionBuffer)) {
                usbChunk = sizeof(usbToNextionBuffer);
            }

            size_t usbReadCount = 0;
            while (usbReadCount < usbChunk && Serial.available() > 0) {
                int incomingByte = Serial.read();
                if (incomingByte < 0) {
                    break;
                }
                usbToNextionBuffer[usbReadCount++] = (uint8_t)incomingByte;
            }

            if (usbReadCount > 0) {
                Serial2.write(usbToNextionBuffer, usbReadCount);
                transferredAnyByte = true;
            }
        }

        // Richtung 2: Antworten oder Bootloader-Daten des Nextion -> USB zum PC. Auch hier
        // arbeiten wir chunk-basiert, damit hohe Baudraten nicht an Byte-fuer-Byte-Overhead
        // scheitern.
        size_t nextionAvailable = (size_t)Serial2.available();
        if (nextionAvailable > 0) {
            size_t nextionChunk = nextionAvailable;
            if (nextionChunk > sizeof(nextionToUsbBuffer)) {
                nextionChunk = sizeof(nextionToUsbBuffer);
            }

            size_t nextionReadCount = 0;
            while (nextionReadCount < nextionChunk && Serial2.available() > 0) {
                int outgoingByte = Serial2.read();
                if (outgoingByte < 0) {
                    break;
                }
                nextionToUsbBuffer[nextionReadCount++] = (uint8_t)outgoingByte;
            }

            if (nextionReadCount > 0) {
                Serial.write(nextionToUsbBuffer, nextionReadCount);
                transferredAnyByte = true;
            }
        }

        // Fuer hohe Flash-Baudraten darf die Schleife nicht permanent schlafen. Deshalb gibt es
        // nur dann einen minimalen 1-Tick-Delay, wenn in diesem Durchlauf wirklich keinerlei
        // Daten anlagen. Bei aktivem Traffic geben wir lediglich den Scheduler frei.
        if (!transferredAnyByte) {
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            taskYIELD();
        }
    }

    // Dieser Mutex-Zugriff beendet die exklusive Serial2-Nutzung wieder sauber, damit Core 0
    // anschliessend regulare Display-Kommandos senden darf.
    xSemaphoreGive(xNexMutex);

    // Nach dem Bridge-Modus kehren wir explizit in den Setup-Kontext zurueck, weil der Einstieg
    // ausschliesslich aus dem Setup-Menue heraus erfolgt.
    currentMode = MODE_SETUP;
}