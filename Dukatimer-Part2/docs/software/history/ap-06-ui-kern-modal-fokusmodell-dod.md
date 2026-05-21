# AP-06 UI-Kern, Modalebene und Fokusmodell (DoD)

## Ziel

Diese DoD konkretisiert AP-06 in technische Regeln fuer:

- globale Modalebene (Fault/Confirm/Wait)
- reproduzierbares Fokusmodell (Touch/Encoder/Modal)
- globalen EventGuard vor Workflow-Dispatch
- blind bedienbare Mindestregeln

## Geltungsbereich

Die Regeln gelten fuer den Input- und UI-Kern in:

- `src/teensy/InputRouterPolicy.*`
- `src/teensy/main.cpp` (Dispatch-Reihenfolge und Snapshot-Wiring)
- `src/teensy/SystemSnapshot.h`
- `src/teensy/UiPresenter.*`
- `src/teensy/LvglUi.*`

## Regel 1: Modale Prioritaet

Modale Prioritaet ist strikt und global:

1. `WorkflowFault`
2. `WorkflowConfirm`
3. `WorkflowWait`
4. `None`

Ableitung:

- `WorkflowFault`: Fault-Lage aus Exposure/Workflow.
- `WorkflowConfirm`: bestaetigungsbeduerftiger Zustand (aktuell SG WaitForFilter).
- `WorkflowWait`: aktive Belichtungsphasen (`PreWait`, `Exposing`, `Paused`, `PostWait`).

## Regel 2: Fokusmodell

Fokus ist deterministisch und nicht implizit:

- Bei aktivem Modal ist Fokus immer `Modal`.
- Ohne Modal hat Touch Vorrang waehrend Aktivitaet plus Hold-Fenster.
- Nach Touch-Hold faellt Fokus auf `Encoder` zurueck.

Prioritaet: `Modal` > `Touch` > `Encoder`.

## Regel 3: Globaler EventGuard

Der EventGuard liegt vor dem Workflow-Dispatch und entscheidet global ueber Zulassung.

### Guard-Zustaende

- `Open`
- `TouchPriority`
- `ConfirmLocked`
- `WaitLocked`
- `FaultLocked`

### Zulassungsmatrix

- `WaitLocked`: nur `Undo` ist zulaessig.
- `ConfirmLocked`: Navigation/Parameterbearbeitung gesperrt; bestaetigende bzw. quittierende Aktionen bleiben zulaessig (`Confirm`, `Start`, `Measure`, `Undo`).
- `FaultLocked`: wie Confirm-Lock, ebenfalls mit zulaessiger Quittierung.
- `TouchPriority` ohne Modal: Encoder-Rotationsereignisse werden blockiert.

## Regel 3.1: Stale-Wait-Recovery

Ein festhaengender globaler Wait-Lock darf nicht unbegrenzt bestehen bleiben.

- Kurzlebige Engine-Phasen (`PreWait`, `PostWait`) muessen innerhalb eines klaren
	Guard-Fensters wieder verlassen werden.
- Bleibt eine solche Wait-Phase unmoeglich lange aktiv, wird nicht still nach
	`Ready` entsperrt, sondern kontrolliert in einen sichtbaren
	`InternalFault` gewechselt.
- Begründung: Bei unklarer Belichtungslage ist ein sicherer Fault-Rueckfall
	zulaessiger als freie Parameterbearbeitung unter moeglicherweise noch aktivem
	Output.
- Die Diagnose muss das Phasenalter sichtbar exportieren.

## Regel 4: UI-Modaldarstellung

Overlay ist global, nicht SG-exklusiv:

- Fault-Modal: klare Fehlernachricht mit Quittierhinweis.
- Confirm-Modal: explizite Bestaetigungsanweisung.
- Wait-Modal: klare Warte-/Sperrhinweise plus Undo-Ausweg.
- Nicht-modale Hinweise bleiben als Kontextoverlay (z. B. Remote stale, parameter dirty).

## Regel 5: Blindbedienungs-Mindestregeln

- Bedienende sehen immer den aktiven Modal-/Guardzustand in den Diagnosezeilen.
- In gesperrten Phasen gibt es genau einen dokumentierten Ausweg (Undo).
- Parameterveraenderung ist in Confirm/Fault/Wait nicht moeglich.
- Fokuswechsel erfolgen nur nach dokumentierter Prioritaetslogik.

## Abnahmechecks

1. Fault provozieren: Overlay zeigt Fault-Hinweis, Guard wird als `FAULT` sichtbar, Rotationsnavigation bleibt gesperrt.
2. SG WaitForFilter provozieren: Overlay zeigt Confirm-Hinweis, Guard `CONF`, nur bestaetigende Aktionen fuehren weiter.
3. Aktive Belichtung starten: Overlay zeigt Wait-Hinweis, Guard `WAIT`, Parameterdrehen blockiert, Undo bricht reproduzierbar ab.
4. Touch halten und Encoder drehen: Guard `TOUCH`, Rotationsereignisse werden nicht parallel an Workflow durchgereicht.
5. Modal verlassen: Fokus faellt reproduzierbar auf Touch (bei Hold) oder Encoder zurueck.
