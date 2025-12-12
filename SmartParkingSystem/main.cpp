/*
 * SMART PARKING SYSTEM - Single File Complete Solution
 * Features:
 * - Asynchronous Ultrasonic Sensor Reading (Interrupt based)
 * - Nearest Neighbor Blinking Logic (Lookup Table)
 * - BLE Control via nRF Connect (System ON/OFF, Blink ON/OFF)
 * - Auto-Reset Timeout (Anti-Ghosting)
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
// 1. PARKING CONTROLLER (HARDWARE LOGIC)
// =================================================================================

// Configuration Structure: Distance -> Blinking Rate
struct BlinkConfig {
    int standard_distance_cm; // Reference distance
    int wait_ticks;           // Blinking speed (0=Solid ON, -1=OFF)
};

class ParkingController {

public:
    // Constructor
    ParkingController(PinName trigPin, PinName echoPin, PinName ledBlinkPin, PinName ledStatusPin) :
        _trig(trigPin),
        _echo(echoPin),
        _ledBlink(ledBlinkPin),   
        _ledStatus(ledStatusPin), 
        _timer()
    {
        // Sensor Interrupt Setup (Asynchronous reading)
        _echo.rise(callback(this, &ParkingController::onEchoRise));
        _echo.fall(callback(this, &ParkingController::onEchoFall));
        
        // Start Timer and set initial state
        _timer.start();
        _ledBlink = 0;
        _ledStatus = 0;
        
        last_valid_reading_time = _timer.elapsed_time().count();
    }

    // --- VARIABLES CONTROLLABLE VIA BLE ---
    bool is_system_active = true;   // Master Switch (Everything OFF/ON)
    bool is_blink_enabled = true;   // Controls only the blinking LED

    // --- READING VARIABLE ---
    volatile int current_distance_cm = 400; // Default: Far away

    // --- MAIN LOOP (to be called every 100ms) ---
    void update_loop() {
        
        // 1. If turned off via BLE, reset everything and exit
        if (!is_system_active) {
            _ledBlink = 0;
            _ledStatus = 0;
            return;
        }

        // 2. Timeout Check (Anti-Ghosting)
        // If we haven't heard an echo for 200ms, reset distance to "Far"
        uint64_t now = _timer.elapsed_time().count();
        if ((now - last_valid_reading_time) > 200000) { 
            current_distance_cm = 400; 
        }

        // 3. Trigger Sensor
        trigger_sensor();

        // 4. Status LED Management (e.g., Green/Red on Display)
        if (current_distance_cm > 0 && current_distance_cm < 200) {
            _ledStatus = 1; // Car present
        } else {
            _ledStatus = 0; // Free
        }

        // 5. Blinking LED Management (Nearest Neighbor)
        if (is_blink_enabled) {
            int best_ticks = get_ticks_for_closest_distance(current_distance_cm);
            apply_blinking_logic(best_ticks);
        } else {
            _ledBlink = 0;
        }
    }

private:
    DigitalOut _trig;
    InterruptIn _echo;
    DigitalOut _ledBlink; 
    DigitalOut _ledStatus;
    Timer _timer;

    volatile uint64_t start_time = 0;
    volatile uint64_t last_valid_reading_time = 0; 
    int blink_counter = 0;

    // ZONE TABLE
    static const int TABLE_SIZE = 5;
    const BlinkConfig blink_table[TABLE_SIZE] = {
        {  10,  0 },  // 10cm  -> Solid ON (STOP)
        {  30,  2 },  // 30cm  -> Fast Blink
        {  60,  5 },  // 60cm  -> Medium Blink
        { 100, 10 },  // 100cm -> Slow Blink
        { 250, -1 }   // 250cm -> OFF
    };

    // Nearest Neighbor Search Algorithm
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

    void apply_blinking_logic(int ticks) {
        if (ticks == -1) { _ledBlink = 0; blink_counter = 0; return; }
        if (ticks == 0)  { _ledBlink = 1; blink_counter = 0; return; }

        blink_counter++;
        if (blink_counter >= ticks) {
            _ledBlink = !_ledBlink;
            blink_counter = 0;
        }
    }

    // Sensor Interrupts
    void onEchoRise() {
        start_time = _timer.elapsed_time().count();
    }

    void onEchoFall() {
        uint64_t now = _timer.elapsed_time().count();
        uint64_t duration = now - start_time;
        int d = (int)(duration / 58);
        
        if (d > 0 && d < 450) {
            current_distance_cm = d;
            last_valid_reading_time = now; // Reset timeout
        }
    }

    void trigger_sensor() {
        _trig = 1;
        wait_us(10);
        _trig = 0;
    }
};

// =================================================================================
// 2. PARKING SERVICE (BLE LOGIC)
// =================================================================================

class ParkingService : public ble::GattServer::EventHandler {
public:
    // Randomly generated UUIDs for your custom service
    const static uint16_t SERVICE_UUID = 0xA000;
    const static uint16_t CHAR_SYSTEM_UUID = 0xA001;
    const static uint16_t CHAR_BLINK_UUID  = 0xA002;

    ParkingService(ParkingController& controller) :
        _controller(controller),
        _system_char(CHAR_SYSTEM_UUID, 1), // Default 1 (ON)
        _blink_char(CHAR_BLINK_UUID, 1),   // Default 1 (ON)
        _parking_service(
            SERVICE_UUID,
            _chars,
            sizeof(_chars) / sizeof(_chars[0])
        )
    {
        _chars[0] = &_system_char;
        _chars[1] = &_blink_char;
    }

    void start(BLE &ble, events::EventQueue &event_queue) {
        _server = &ble.gattServer();
        _server->addService(_parking_service);
        _server->setEventHandler(this);
        
        printf("Parking Service started.\r\n");
    }

protected:
    // Callback when writing from the nRF Connect App
    void onDataWritten(const GattWriteCallbackParams &params) override {
        
        // Case 1: Write on SYSTEM CONTROL (Master Switch)
        if (params.handle == _system_char.getValueHandle() && params.len == 1) {
            bool val = params.data[0] != 0;
            _controller.is_system_active = val;
            printf("BLE Write: System Active -> %d\r\n", val);
        }
        
        // Case 2: Write on BLINK CONTROL (Only Blinking LED)
        else if (params.handle == _blink_char.getValueHandle() && params.len == 1) {
            bool val = params.data[0] != 0;
            _controller.is_blink_enabled = val;
            printf("BLE Write: Blink Enabled -> %d\r\n", val);
        }
    }

private:
    ParkingController& _controller; // Reference to hardware controller
    GattServer *_server = nullptr;

    // Template Helper to define Read/Write characteristics
    template<typename T>
    class RWChar : public GattCharacteristic {
    public:
        RWChar(const UUID &uuid, const T& initial) :
            GattCharacteristic(uuid, &_value, sizeof(_value), sizeof(_value),
                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ),
            _value(initial) {}
    private:
        uint8_t _value;
    };

    RWChar<uint8_t> _system_char;
    RWChar<uint8_t> _blink_char;
    GattCharacteristic* _chars[2];
    GattService _parking_service;
};

// =================================================================================
// 3. MAIN (SETUP AND CONNECTION)
// =================================================================================

// Define the correct board PINs here
// Example: Trigger on D10, Echo on D11, Blinking LED on LED1, Status LED on D9
ParkingController myParking(D10, D11, LED1, D9);

int main()
{
    mbed_trace_init();

    BLE &ble = BLE::Instance();
    events::EventQueue event_queue;

    // BLE Service Instance (passing the controller as reference)
    ParkingService parking_service_ble(myParking);

    // Standard Mbed process to handle connection/advertising
    GattServerProcess ble_process(event_queue, ble);

    ble_process.on_init([&](BLE &ble, events::EventQueue &q) {
        
        // 1. Start BLE services
        parking_service_ble.start(ble, q);
        
        // 2. START PARKING LOGIC
        // The event queue will call "update_loop" every 100 milliseconds
        // This works regardless of whether Bluetooth is connected or not
        q.call_every(100ms, callback(&myParking, &ParkingController::update_loop));

        printf("System started. Waiting for BLE...\r\n");
    });

    ble_process.start();

    return 0;
}