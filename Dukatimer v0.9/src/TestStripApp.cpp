#include "TestStripApp.h"
#include "SystemContext.h"
#include "HardwareManager.h"
#include "PaperManager.h"

/* =============================================================================
 * TestStripApp.cpp - DUKATIMER BETA (v0.916.6)
 * * Implementierung des Probestreifen-Modus.
 * * (+ FIX: Explizite Includes im CPP zur Härtung gegen schwache Header-Ketten)
 * * (+ FIX: Konsistente Variablenbenennung 'st' zur Vermeidung von Scope-Errors)
 * * (+ FIX: UI-Flickering und State-Freeze behoben)
 * ========================================================================== */

TestStripApp::TestStripApp(SystemContext *ctx, HardwareManager *hw, PaperManager *pm)
    : _ctx(ctx), _hw(hw), _pm(pm), _lastMs(0), _step(0)
{
}

TestStripApp::~TestStripApp() {}

void TestStripApp::onEnter()
{
    Serial.printf("[TestStrip] onEnter()\n");
    _lastMs = millis();
    _step = 0;

    // Initialer Push des sauberen Grundzustands
    AppSharedState st{};
    st.activeStateMode = MODE_TEST_STRIP;
    st.testStrip.activeStep = _step;
    st.testStrip.lastLux = 0.0f;

    if (_ctx)
    {
        _ctx->setAppState(st);
    }
}

void TestStripApp::handleInput(int event)
{
    if (!_ctx || !_hw)
        return;

    // Event 1 = next step, 0 = abort
    if (event == EV_MENU_CLICK)
    {
        _step++;
        Serial.printf("[TestStrip] Next step %d\n", _step);

        // FIX (Flickering): Sofortiger Snapshot des aktuellen Lux-Werts
        float currentLux = 0.0f;
        HardwareStatus hs;
        if (_ctx->getStatus(hs))
        {
            currentLux = hs.headLux;
        }

        AppSharedState st{};
        st.activeStateMode = MODE_TEST_STRIP;
        st.testStrip.activeStep = _step;
        st.testStrip.lastLux = currentLux;
        _ctx->setAppState(st);

        _hw->playBeep(BEEP_TICK);
    }
    else if (event == EV_ABORT)
    {
        _step = 0;
        Serial.printf("[TestStrip] Aborted\n");

        // FIX (State-Freeze): Sofortiger Reset-Push an das UI
        AppSharedState st{};
        st.activeStateMode = MODE_TEST_STRIP;
        st.testStrip.activeStep = _step;
        st.testStrip.lastLux = 0.0f;
        _ctx->setAppState(st);

        _hw->playBeep(BEEP_WARN);
    }
}

void TestStripApp::onUpdate()
{
    if (!_ctx)
        return;

    unsigned long now = millis();
    if (now - _lastMs < INTERVAL_MS)
        return;
    _lastMs = now;

    // Reguläres Polling des Messwerts
    HardwareStatus hs;
    if (_ctx->getStatus(hs))
    {
        Serial.printf("[TestStrip] step=%d HeadLux=%.2f\n", _step, hs.headLux);

        AppSharedState st{};
        st.activeStateMode = MODE_TEST_STRIP;
        st.testStrip.activeStep = _step;
        st.testStrip.lastLux = hs.headLux;
        _ctx->setAppState(st);
    }
}

void TestStripApp::onExit()
{
    Serial.printf("[TestStrip] onExit()\n");
    // Beim Verlassen wird die Union durch den AppManager/Context bereinigt.
}