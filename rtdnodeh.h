
#pragma once

#include <cstdint>

// ============================================================
//  Pin Definitions  (update to match your wiring)
// ============================================================
// constexpr uint PIN_BRAKE_SWITCH  = 2;   // Digital IN  – active HIGH when brake pressed
// constexpr uint PIN_RTD_BUTTON    = 20;//9/20/21/32   // Digital IN  – active HIGH when button pressed 
// constexpr uint PIN_SHUTDOWN_LOOP = 4;   // Digital IN  – HIGH = shutdown circuit closed (safe)
constexpr uint PIN_SEVCON_ENABLE = 20;   // Digital OUT – HIGH = allow torque, LOW = inhibit
// constexpr uint PIN_RTD_BUZZER    = 6;   // Digital OUT – drive speaker/buzzer for RTD tone

// ============================================================
//  CAN message IDs  (align with DBC)
// ============================================================
constexpr uint32_t CAN_ID_VEHICLE_STATE = 0x100;  // Broadcast vehicle state
constexpr uint32_t CAN_ID_BMS_STATUS    = 0x200;  // Received from BMS

// ============================================================
//  Timing
// ============================================================
// constexpr uint32_t RTD_BUZZER_DURATION_MS = 2000;  // 2-second RTD tone
constexpr uint32_t LOOP_PERIOD_MS         = 10;    // 10 ms main loop tick

// ============================================================
//  Vehicle State Machine
// ============================================================
enum class VehicleState : uint8_t {
    OFF        = 0,
    LV_ON      = 1,
    PRECHARGE  = 2,
    HV_READY   = 3,
    RTD        = 4,   // Ready-To-Drive
    DRIVE      = 5,
    FAULT      = 6
};

const char* vehicleStateToString(VehicleState state);