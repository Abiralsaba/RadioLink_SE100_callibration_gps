// Last_Wish.h — ESP32 Last Wish
// Servo Position Preservation on Sudden Power Loss
// "When power dies, the ESP32 fulfils its last wish — saving its position before the final reset."
//
// Optimized for minimum latency from power-cut detection to NVS write
// ISR → Context switch → Snapshot → NVS write happens as fast as possible
//
// GitHub: https://github.com/Thuha77/ESP32_Last_Wish
// License: MIT

#pragma once
#include <Arduino.h>
#include <Preferences.h>

static Preferences _lw_prefs;
static TaskHandle_t _lw_saveHandle = NULL;
static volatile int* _lw_curPtr = nullptr;
static volatile bool _lw_saveDone = false;

// ================================================
// Ultra-light ISR - ONLY wakes the high-priority task
// Minimal logic → fastest possible response
// ================================================
void IRAM_ATTR _lw_powerCutISR() {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(_lw_saveHandle, &woken);
    portYIELD_FROM_ISR(woken);
}

// ================================================
// Save task - Highest priority on Core 1
// Wakes up quickly and writes to NVS with minimal delay
// ================================================
void _lw_saveTask(void* param) {
    for (;;) {
        // Sleep with zero CPU usage until ISR wakes us
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        _lw_saveDone = true;

        // Take snapshot of current position (very fast)
        int snapshot = *_lw_curPtr;

        // Write to NVS (Preferences)
        _lw_prefs.putUInt("pwmVal", (uint32_t)snapshot);

        // Optional: small safety delay for NVS internal operations (usually not needed)
         vTaskDelay(2);

        // Task has done its job - suspend forever
        vTaskSuspend(NULL);
    }
}

// ================================================
// LastWish_begin() — Call FIRST in setup()
//   pin    : GPIO connected to voltage-divider for power-cut detection
//   cur    : volatile int that holds the live servo PWM value (µs)
// ================================================
void LastWish_begin(int pin, volatile int& cur) {
    _lw_curPtr = &cur;

    // Initialize NVS namespace
    _lw_prefs.begin("lw_servo", false);

    // Create high-priority save task on Core 1
    xTaskCreatePinnedToCore(
        _lw_saveTask,           // task function
        "LastWish",             // task name
        4096,                   // stack size in bytes
        NULL,                   // parameter
        24,                     // priority (highest user priority)
        &_lw_saveHandle,        // task handle
        1                       // Core 1 (better for low interference with WiFi)
    );

    // Setup power-cut detection pin
    pinMode(pin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(pin), _lw_powerCutISR, FALLING);
}

// ================================================
// LastWish_load() — Call AFTER LastWish_begin() in setup()
//   defaultVal : value to return on first boot or invalid saved value
// ================================================
int LastWish_load(int defaultVal) {
    uint32_t v = _lw_prefs.getUInt("pwmVal", (uint32_t)defaultVal);
    // Basic range validation (adjust min/max according to your servo)
    //return (v >= 800 && v <= 1650) ? (int)v : defaultVal;
    return (int)v;
}

// ================================================
// LastWish_clear() — Erase saved position (for factory reset or testing)
// ================================================
void LastWish_clear() {
    _lw_prefs.clear();
    Serial.println("[LastWish] Saved position cleared.");
}