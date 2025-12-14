/*
 * SMART PARKING SYSTEM - Single File Complete Solution
 *
 * Features:
 * - Asynchronous ultrasonic distance measurement (interrupt-based)
 * - Distance-dependent blinking logic (nearest-neighbor lookup table)
 * - BLE control via nRF Connect (system ON/OFF, blinking ON/OFF)
 * - Measurement timeout (anti-ghosting mechanism)
 *
 * Target platform: Mbed OS + BLE
 */

#include "mbed.h"
#include "platform/Callback.h"
#include "events/EventQueue.h"
#include "ble/BLE.h"
#include "gatt_server_process.h"
#include "mbed-trace/mbed_trace.h"
#include <cmath>
#include <cstdint>

using mbed::callback;
using namespace std::literals::chrono_literals;

// =================================================================================
// DATA STRUCTURES
// =================================================================================

/*
 * Blink configuration entry.
 * Each entry represents a reference distance and the corresponding
 * blinking speed (expressed in scheduler ticks).
 */
struct BlinkConfig {
    int standard_distance_cm; // Reference distance (center of a distance zone)
    int wait_ticks;           // Blinking rate: 0 = solid ON, -1 = OFF
};

/*
 * Shared system state for a single parking space.
 * This structure acts as a central context and is shared
 * between all system components via references.
 */
struct ParkSpaceConfig {
    bool system_active = true;   // Master enable/disable flag (set via BLE)
    bool blink_enabled = true;   // Enable/disable blinking LED (set via BLE)

    int  distance_cm = 400;      // Last measured distance (cm)
    bool occupied = false;       // Parking space occupancy state

    int blink_ticks = -1;        // Current blinking mode
};

// =================================================================================
// ULTRASONIC SENSOR (HARDWARE ABSTRACTION)
// =================================================================================

/*
 * UltrasonicSensor
 *
 * Handles low-level ultrasonic distance measurement using:
 * - Trigger pin (output)
 * - Echo pin (interrupt input)
 *
 * Measurement is fully asynchronous and interrupt-driven.
 */
class UltrasonicSensor {
public:
    UltrasonicSensor(PinName trig, PinName echo)
        : _trig(trig), _echo(echo)
    {
        // Register interrupt handlers for echo signal
        _echo.rise(callback(this, &UltrasonicSensor::onRise));
        _echo.fall(callback(this, &UltrasonicSensor::onFall));

        // Start internal timer used for pulse width measurement
        _timer.start();
    }

    /*
     * Sends a 10 µs trigger pulse to start a measurement.
     * Called periodically from the EventQueue.
     */
    void trigger() {
        _trig = 1;
        wait_us(10);
        _trig = 0;
    }

    /*
     * Returns true if a valid measurement has been received
     * within the specified timeout window.
     */
    bool hasFreshReading(uint64_t now_us) const {
        return (now_us - _last_valid_time) < 200000; // 200 ms timeout
    }

    /*
     * Returns the last measured distance in centimeters.
     */
    int getDistanceCm() const {
        return _distance_cm;
    }

private:
    DigitalOut   _trig;   // Trigger output pin
    InterruptIn _echo;    // Echo input pin (interrupt-driven)
    Timer       _timer;  // Timer for pulse width measurement

    volatile uint64_t _start_time = 0;       // Echo pulse start time
    volatile uint64_t _last_valid_time = 0;  // Timestamp of last valid measurement
    volatile int      _distance_cm = 400;    // Cached distance value

    /*
     * Interrupt handler: echo signal rising edge.
     * Stores the start time of the echo pulse.
     */
    void onRise() {
        _start_time = _timer.elapsed_time().count();
    }

    /*
     * Interrupt handler: echo signal falling edge.
     * Computes pulse duration and converts it to distance.
     */
    void onFall() {
        uint64_t now = _timer.elapsed_time().count();
        uint64_t dur = now - _start_time;
        int d = dur / 58; // Conversion from µs to cm

        if (d > 0 && d < 450) {
            _distance_cm = d;
            _last_valid_time = now;
        }
    }
};

// =================================================================================
// PARKING LOGIC (DECISION MAKING)
// =================================================================================

/*
 * ParkingLogic
 *
 * Implements all decision-making logic:
 * - occupancy detection
 * - blinking mode selection
 *
 * Does not interact directly with hardware.
 */
class ParkingLogic {
public:
    ParkingLogic(ParkSpaceConfig& parkingSpaceState)
        : _parkingSpaceState(parkingSpaceState) {}

    /*
     * Periodic update function.
     * Called from the EventQueue at a fixed interval.
     */
    void update(uint64_t now_us, const UltrasonicSensor& sensor) {

        // System disabled via BLE -> reset outputs
        if (!_parkingSpaceState.system_active) {
            _parkingSpaceState.occupied = false;
            _parkingSpaceState.blink_ticks = -1;
            return;
        }

        // No recent measurement -> assume no object present
        if (!sensor.hasFreshReading(now_us)) {
            _parkingSpaceState.distance_cm = 400;
            _parkingSpaceState.occupied = false;
            _parkingSpaceState.blink_ticks = -1;
            return;
        }

        // Update distance and occupancy state
        _parkingSpaceState.distance_cm = sensor.getDistanceCm();
        _parkingSpaceState.occupied =
            (_parkingSpaceState.distance_cm < 200);

        // Select blinking mode using nearest-neighbor lookup
        _parkingSpaceState.blink_ticks =
            get_ticks_for_closest_distance(_parkingSpaceState.distance_cm);
    }

private:
    ParkSpaceConfig& _parkingSpaceState;

    // Lookup table for distance-based blinking behavior
    static const int TABLE_SIZE = 7;
    const BlinkConfig blink_table[TABLE_SIZE] = {
        {  10,  0 },   // Very close -> solid ON
        {  30,  2 },   // Fast blink
        {  60,  5 },   // Medium blink
        { 100,  8 },
        { 150, 14 },
        { 200, 20 },
        { 250, -1 }    // Far / no object -> OFF
    };

    /*
     * Nearest-neighbor selection algorithm.
     * Chooses the blinking configuration whose reference distance
     * is closest to the current measured distance.
     */
    int get_ticks_for_closest_distance(int real_dist) {
        int best_ticks = -1;
        int min_diff = 9999;

        for (int i = 0; i < TABLE_SIZE; i++) {
            int diff = abs(real_dist - blink_table[i].standard_distance_cm);
            if (diff < min_diff) {
                min_diff = diff;
                best_ticks = blink_table[i].wait_ticks;
            }
        }
        return best_ticks;
    }
};

// =================================================================================
// LED OUTPUT CONTROLLERS
// =================================================================================

/*
 * Status LED controller.
 * Indicates whether the parking space is free or occupied.
 */
class StatusLed {
public:
    StatusLed(PinName pin) : _led(pin) {}

    void update(const ParkSpaceConfig& state) {
        if (!state.system_active) {
            _led = 0;
        } else {
            _led = state.occupied ? 0 : 1; // ON when free
        }
    }

private:
    DigitalOut _led;
};

/*
 * Blinking LED controller.
 * Visualizes the distance to the detected object.
 */
class BlinkLed {
public:
    BlinkLed(PinName pin) : _led(pin) {}

    void tick(const ParkSpaceConfig& state) {

        // Disabled either globally or via BLE
        if (!state.system_active || !state.blink_enabled) {
            _led = 0;
            _counter = 0;
            return;
        }

        if (state.blink_ticks == -1) {
            _led = 0;
            _counter = 0;
        }
        else if (state.blink_ticks == 0) {
            _led = 1; // Solid ON
        }
        else {
            _counter++;
            if (_counter >= state.blink_ticks) {
                _led = !_led;
                _counter = 0;
            }
        }
    }

private:
    DigitalOut _led;
    int _counter = 0;
};

// =================================================================================
// BLE SERVICE (REMOTE CONTROL INTERFACE)
// =================================================================================

/*
 * ParkingBLEService
 *
 * Provides BLE characteristics to remotely control:
 * - system ON/OFF
 * - blinking LED enable/disable
 */
class ParkingBLEService : public ble::GattServer::EventHandler {
public:
    static const uint16_t SERVICE_UUID = 0xA000;
    static const uint16_t CHAR_SYSTEM  = 0xA001;
    static const uint16_t CHAR_BLINK   = 0xA002;

    ParkingBLEService(ParkSpaceConfig& state)
        : _state(state),
          _sys_char(CHAR_SYSTEM, 1),
          _blink_char(CHAR_BLINK, 1),
          _service(SERVICE_UUID, _chars, 2)
    {
        _chars[0] = &_sys_char;
        _chars[1] = &_blink_char;
    }

    void start(BLE& ble) {
        _server = &ble.gattServer();
        _server->addService(_service);
        _server->setEventHandler(this);
    }

    /*
     * Callback invoked when a BLE client writes to a characteristic.
     */
    void onDataWritten(const GattWriteCallbackParams& p) override {
        if (p.handle == _sys_char.getValueHandle()) {
            _state.system_active = (p.data[0] != 0);
        }
        else if (p.handle == _blink_char.getValueHandle()) {
            _state.blink_enabled = (p.data[0] != 0);
        }
    }

private:
    ParkSpaceConfig& _state;
    GattServer* _server;

    template<typename T>
    class RWChar : public GattCharacteristic {
    public:
        RWChar(uint16_t uuid, T v)
            : GattCharacteristic(uuid, &_val, sizeof(_val), sizeof(_val),
                BLE_GATT_CHAR_PROPERTIES_READ |
                BLE_GATT_CHAR_PROPERTIES_WRITE),
              _val(v) {}
    private:
        uint8_t _val;
    };

    RWChar<uint8_t> _sys_char;
    RWChar<uint8_t> _blink_char;
    GattCharacteristic* _chars[2];
    GattService _service;
};

// =================================================================================
// MAIN APPLICATION ENTRY POINT
// =================================================================================

int main()
{
    // Shared system state
    ParkSpaceConfig parkingSpaceState;

    BLE& ble = BLE::Instance();
    events::EventQueue q;

    // Hardware components
    UltrasonicSensor usSensor(D10, D11);
    ParkingLogic logic(parkingSpaceState);
    StatusLed statusLed(D9);
    BlinkLed blinkLed(LED1);

    // BLE service
    ParkingBLEService bleService(parkingSpaceState);
    GattServerProcess bleProcess(q, ble);

    bleProcess.on_init([&](BLE&, events::EventQueue& q) {

        // Start BLE service
        bleService.start(ble);

        // Main control loop (sensor + logic + status LED)
        q.call_every(100ms, [&] {
            usSensor.trigger();
            uint64_t now = Kernel::get_ms_count() * 1000;
            logic.update(now, usSensor);
            statusLed.update(parkingSpaceState);
        });

        // Fast loop for blinking LED timing
        q.call_every(20ms, [&] {
            blinkLed.tick(parkingSpaceState);
        });
    });

    bleProcess.start();
    return 0;
}
