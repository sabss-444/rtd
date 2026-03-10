/**
 * rtd_node.cpp
 *
 * Ready-To-Drive (RTD) firmware for RP2040 / Raspberry Pi Pico
 * Formula Student EV – Dashboard Pico Node
 *
 * Responsibilities:
 *   1. Track vehicle state (OFF → LV_ON → PRECHARGE → HV_READY → RTD → DRIVE)
 *   2. Assert / revoke Sevcon ENABLE pin based on RTD conditions
 *   3. Sound buzzer when RTD is confirmed (rules requirement)
 *   4. Broadcast vehicle state over CAN for dash / DAQ
 *
 * RTD Conditions (all must be true simultaneously):
 *   - Brake switch pressed
 *   - RTD button pressed
 *   - Shutdown circuit closed
 *   - BMS status OK  (received via CAN)
 *   - No active faults
 *
 * Sevcon ENABLE pin:
 *   HIGH → Sevcon allowed to produce torque
 *   LOW  → Sevcon inhibited (no torque)
 *
 * Build: RP2040 SDK  (pico_stdlib, hardware_gpio, hardware_timer)
 *        Add your libcan/CAN driver to CMakeLists.txt
 */

#include "rtd_node.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// ---------------------------------------------------------------------------
//  Stub: replace with your actual libcan calls
// ---------------------------------------------------------------------------
namespace can {

struct Frame {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
};

/** Receive a CAN frame – returns true if a new frame was available. */
bool receive(Frame& out_frame) {
    // TODO: wire up to libcan  e.g. return libcan_receive(&out_frame);
    (void)out_frame;
    return false;
}

/** Transmit a CAN frame. */
void transmit(const Frame& frame) {
    // TODO: wire up to libcan  e.g. libcan_transmit(&frame);
    (void)frame;
}

} // namespace can

// ---------------------------------------------------------------------------
//  Helper: vehicleStateToString
// ---------------------------------------------------------------------------
const char* vehicleStateToString(VehicleState state) {
    switch (state) {
        case VehicleState::OFF:       return "OFF";
        case VehicleState::LV_ON:     return "LV_ON";
        case VehicleState::PRECHARGE: return "PRECHARGE";
        case VehicleState::HV_READY:  return "HV_READY";
        case VehicleState::RTD:       return "RTD";
        case VehicleState::DRIVE:     return "DRIVE";
        case VehicleState::FAULT:     return "FAULT";
        default:                      return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------------
//  RTD Node class
// ---------------------------------------------------------------------------
class RTDNode {
public:
    RTDNode() = default;

    void init() {
        // --- GPIO inputs ---
        gpio_init(PIN_BRAKE_SWITCH);
        gpio_set_dir(PIN_BRAKE_SWITCH, GPIO_IN);
        gpio_pull_down(PIN_BRAKE_SWITCH);   // floating → LOW when open

        gpio_init(PIN_RTD_BUTTON);
        gpio_set_dir(PIN_RTD_BUTTON, GPIO_IN);
        gpio_pull_down(PIN_RTD_BUTTON);

        gpio_init(PIN_SHUTDOWN_LOOP);
        gpio_set_dir(PIN_SHUTDOWN_LOOP, GPIO_IN);
        gpio_pull_down(PIN_SHUTDOWN_LOOP);

        // --- GPIO outputs (safe defaults first) ---
        gpio_init(PIN_SEVCON_ENABLE);
        gpio_set_dir(PIN_SEVCON_ENABLE, GPIO_OUT);
        setSevconEnable(false);             // inhibit on boot

        gpio_init(PIN_RTD_BUZZER);
        gpio_set_dir(PIN_RTD_BUZZER, GPIO_OUT);
        gpio_put(PIN_RTD_BUZZER, 0);

        m_state           = VehicleState::LV_ON;  // Pico only runs when LV is on
        m_bms_ok          = false;
        m_buzzer_end_ms   = 0;
        m_last_state      = VehicleState::OFF;
    }

    /** Call this every LOOP_PERIOD_MS. */
    void tick() {
        readInputs();
        processCAN();
        updateStateMachine();
        handleBuzzer();
        broadcastState();
    }

private:
    // -----------------------------------------------------------------------
    //  Internal state
    // -----------------------------------------------------------------------
    VehicleState m_state;
    VehicleState m_last_state;

    // Raw inputs (updated each tick)
    bool m_brake_pressed    = false;
    bool m_rtd_button       = false;
    bool m_shutdown_ok      = false;
    bool m_bms_ok           = false;

    // Buzzer
    uint32_t m_buzzer_end_ms = 0;

    // -----------------------------------------------------------------------
    //  Read all digital inputs
    // -----------------------------------------------------------------------
    void readInputs() {
        m_brake_pressed = gpio_get(PIN_BRAKE_SWITCH);
        m_rtd_button    = gpio_get(PIN_RTD_BUTTON);
        m_shutdown_ok   = gpio_get(PIN_SHUTDOWN_LOOP);
    }

    // -----------------------------------------------------------------------
    //  Poll CAN bus for BMS status and other relevant messages
    // -----------------------------------------------------------------------
    void processCAN() {
        can::Frame frame{};
        while (can::receive(frame)) {
            if (frame.id == CAN_ID_BMS_STATUS && frame.dlc >= 1) {
                // Byte 0: 0x01 = BMS OK, anything else = not OK
                m_bms_ok = (frame.data[0] == 0x01);
            }
            // Add other CAN message handling here (inverter status, etc.)
        }
    }

    // -----------------------------------------------------------------------
    //  Check all RTD pre-conditions
    // -----------------------------------------------------------------------
    bool rtdConditionsMet() const {
        return m_brake_pressed
            && m_rtd_button
            && m_shutdown_ok
            && m_bms_ok;
    }

    // -----------------------------------------------------------------------
    //  State machine
    // -----------------------------------------------------------------------
    void updateStateMachine() {
        VehicleState next = m_state;

        switch (m_state) {

            // -----------------------------------------------------------------
            case VehicleState::LV_ON:
                // Waiting for HV precharge – in this demo we move straight to
                // HV_READY once the shutdown loop is healthy.
                // In a full system, PRECHARGE state would be managed by the
                // Sevcon / a separate HV controller.
                if (m_shutdown_ok) {
                    next = VehicleState::HV_READY;
                }
                break;

            // -----------------------------------------------------------------
            case VehicleState::HV_READY:
                // Stay here until operator performs RTD sequence
                if (rtdConditionsMet()) {
                    next = VehicleState::RTD;
                }
                // If shutdown loop opens, drop back
                if (!m_shutdown_ok) {
                    next = VehicleState::LV_ON;
                }
                break;

            // -----------------------------------------------------------------
            case VehicleState::RTD:
                // Momentary state – sound buzzer then move to DRIVE
                // (buzzer is triggered in the transition handler below)
                next = VehicleState::DRIVE;
                break;

            // -----------------------------------------------------------------
            case VehicleState::DRIVE:
                // Revoke drive if any safety condition fails
                if (!m_shutdown_ok || !m_bms_ok) {
                    next = VehicleState::FAULT;
                }
                // Operator can also revoke by releasing brake+button simultaneously
                // (optional – add further rules here if required)
                break;

            // -----------------------------------------------------------------
            case VehicleState::FAULT:
                // Latch in FAULT until power-cycle or explicit reset
                // (expand with CAN fault-clear message if needed)
                setSevconEnable(false);
                break;

            default:
                break;
        }

        // -------------------------------------------------------------------
        //  Handle state transitions
        // -------------------------------------------------------------------
        if (next != m_state) {
            onStateExit(m_state);
            m_last_state = m_state;
            m_state      = next;
            onStateEnter(m_state);
        }
    }

    void onStateExit(VehicleState state) {
        (void)state;
        // Add any exit actions here
    }

    void onStateEnter(VehicleState state) {
        switch (state) {
            case VehicleState::RTD:
                // Sound the RTD buzzer (rules: audible signal required)
                triggerBuzzer();
                break;

            case VehicleState::DRIVE:
                // Assert Sevcon ENABLE – inverter now allowed to produce torque
                setSevconEnable(true);
                break;

            case VehicleState::FAULT:
                // Immediately inhibit torque
                setSevconEnable(false);
                break;

            case VehicleState::HV_READY:
            case VehicleState::LV_ON:
                setSevconEnable(false);
                break;

            default:
                break;
        }
    }

    // -----------------------------------------------------------------------
    //  Sevcon ENABLE pin control
    // -----------------------------------------------------------------------
    void setSevconEnable(bool enable) {
        gpio_put(PIN_SEVCON_ENABLE, enable ? 1 : 0);
    }

    // -----------------------------------------------------------------------
    //  Buzzer / speaker
    // -----------------------------------------------------------------------
    void triggerBuzzer() {
        gpio_put(PIN_RTD_BUZZER, 1);
        m_buzzer_end_ms = to_ms_since_boot(get_absolute_time()) + RTD_BUZZER_DURATION_MS;
    }

    void handleBuzzer() {
        if (m_buzzer_end_ms != 0) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now >= m_buzzer_end_ms) {
                gpio_put(PIN_RTD_BUZZER, 0);
                m_buzzer_end_ms = 0;
            }
        }
    }

    // -----------------------------------------------------------------------
    //  Broadcast vehicle state over CAN  (for dash / DAQ / logging)
    // -----------------------------------------------------------------------
    void broadcastState() {
        // Only send when state changes to avoid flooding the bus
        if (m_state == m_last_state) return;

        can::Frame frame{};
        frame.id      = CAN_ID_VEHICLE_STATE;
        frame.dlc     = 3;
        frame.data[0] = static_cast<uint8_t>(m_state);
        frame.data[1] = m_brake_pressed  ? 0x01 : 0x00;
        frame.data[2] = m_bms_ok         ? 0x01 : 0x00;

        can::transmit(frame);

        // Update last_state to prevent re-sending until next change
        m_last_state = m_state;
    }
};

// ---------------------------------------------------------------------------
//  Entry point
// ---------------------------------------------------------------------------
int main() {
    stdio_init_all();

    RTDNode rtd;
    rtd.init();

    while (true) {
        rtd.tick();
        sleep_ms(LOOP_PERIOD_MS);
    }

    return 0;
}


