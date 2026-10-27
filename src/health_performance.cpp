#include "health_performance.h"
#include "health.h"
#include "main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <atomic>

// Works with the existing Arduino-ESP32 2.x FreeRTOS build; requires neither
// configGENERATE_RUN_TIME_STATS nor configUSE_TRACE_FACILITY.
// Idle hook counts offer *relative utilization estimates*, not per-task CPU time.
namespace {
std::atomic<uint32_t> idleCount[2] = {};
bool idle0() { idleCount[0].fetch_add(1, std::memory_order_relaxed); return true; }
bool idle1() { idleCount[1].fetch_add(1, std::memory_order_relaxed); return true; }
bool initialized = false;
bool hooksOk[2] = {false, false};
uint32_t priorCount[2] = {};
uint32_t peakRate[2] = {}; // Maximum observed idle cycles per second.
uint32_t priorMs = 0;
uint32_t windowMs = 0;
uint32_t sampleDelta[2] = {};
bool ready = false;
constexpr uint32_t kIntervalMs = 10000;

void initHooks() {
    if (initialized) return;
    hooksOk[0] = (esp_register_freertos_idle_hook_for_cpu(idle0, 0) == ESP_OK);
    hooksOk[1] = (esp_register_freertos_idle_hook_for_cpu(idle1, 1) == ESP_OK);
    for (int i=0;i<2;i++) priorCount[i] = idleCount[i].load(std::memory_order_relaxed);
    priorMs = millis();
    initialized = true;
}

void printTask(Stream& out, const char* name, const TaskHealthMetrics& t) {
    if (!t.isRunning) return;
    out.printf("  %-17s allocated=%lu B  min-free=%lu B  stack-used=%.1f%%\n",
        name, (unsigned long)t.allocatedStackSize,
        (unsigned long)t.minFreeStack, t.stackUtilizationPercent);
}
}

void health_performance_sample() {
    initHooks();
    const uint32_t now = millis();
    const uint32_t elapsed = now - priorMs;
    if (elapsed < kIntervalMs) return;
    windowMs = elapsed;
    for (int i=0;i<2;i++) {
        const uint32_t value = idleCount[i].load(std::memory_order_relaxed);
        sampleDelta[i] = value - priorCount[i]; // uint32_t wrap-safe subtraction.
        priorCount[i] = value;
        const uint32_t rate = (uint32_t)((uint64_t)sampleDelta[i] * 1000ULL / elapsed);
        if (rate > peakRate[i]) peakRate[i] = rate;
    }
    priorMs = now;
    ready = true;
}

void health_performance_print(Stream& out) {
    out.println("=== PERFORMANCE (idle-hook estimate) ===");
    if (!initialized) initHooks();
    if (!hooksOk[0] || !hooksOk[1]) {
        out.printf("Idle-hook installation: core0=%s core1=%s\n",
            hooksOk[0] ? "OK" : "FAILED", hooksOk[1] ? "OK" : "FAILED");
    }
    if (!ready) {
        out.println("Sampling: no completed 10-second window yet.");
    } else {
        out.printf("Sample duration: %lu ms\n", (unsigned long)windowMs);
        for (int i=0;i<2;i++) {
            if (!hooksOk[i] || peakRate[i] == 0) {
                out.printf("Core %d: idle estimate unavailable\n", i);
                continue;
            }
            const float current = (float)sampleDelta[i] * 1000.0f / (float)windowMs;
            float idlePct = (current / (float)peakRate[i]) * 100.0f;
            if (idlePct > 100) idlePct = 100;
            out.printf("Core %d: idle ~%.1f%%  busy ~%.1f%% (relative to best observed idle)\n",
                       i, idlePct, 100.0f - idlePct);
        }
        out.println("Estimates are NOT exact CPU utilization: baseline improves after an idle period.");
    }
    out.println("Task stack high-water marks (maintained by health.cpp):");
    const SystemHealthMetrics m = health_getHealthMetrics();
    printTask(out, "RecordTask", m.recordTaskHealth);
    printTask(out, "NetworkTask", m.uploadTaskHealth);
    printTask(out, "SerialTask", m.serialTaskHealth);
    printTask(out, "MaintenanceTask", m.maintenanceTaskHealth);
    printTask(out, "WebServer", m.webServerTaskHealth);
    out.printf("Heap: free=%lu B  minimum=%lu B  largest-block=%lu B\n",
               (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
               (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    out.printf("PSRAM: free=%lu B  total=%lu B\n",
               (unsigned long)ESP.getFreePsram(), (unsigned long)ESP.getPsramSize());
}
