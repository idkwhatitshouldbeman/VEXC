/**
 * @file mock_pros.hpp
 * @brief Mock PROS API for testing without hardware
 * 
 * This file provides mock implementations of PROS API calls
 * so you can test your robot code logic without VEX hardware.
 */

#ifndef MOCK_PROS_HPP
#define MOCK_PROS_HPP

#include <cstdint>
#include <cmath>
#include <atomic>
#include <vector>
#include <string>
#include <map>

// Mock PROS namespace
namespace pros {
    namespace v5 {
        enum class MotorGears {
            invalid = 0,
            red = 100,      // 100 RPM
            green = 200,    // 200 RPM
            blue = 600     // 600 RPM
        };
        
        enum class MotorUnits {
            invalid = 0,
            degrees = 1,
            rotations = 2
        };
        
        enum class MotorBrake {
            coast = 0,
            brake = 1,
            hold = 2
        };
    }
    
    enum motor_gearset_e {
        E_MOTOR_GEAR_RED = 100,
        E_MOTOR_GEAR_GREEN = 200,
        E_MOTOR_GEAR_BLUE = 600
    };
    
    enum motor_brake_mode_e {
        E_MOTOR_BRAKE_COAST = 0,
        E_MOTOR_BRAKE_BRAKE = 1,
        E_MOTOR_BRAKE_HOLD = 2
    };
    
    enum controller_digital_e {
        E_CONTROLLER_DIGITAL_L1 = 6,
        E_CONTROLLER_DIGITAL_L2 = 7,
        E_CONTROLLER_DIGITAL_R1 = 8,
        E_CONTROLLER_DIGITAL_R2 = 9,
        E_CONTROLLER_DIGITAL_UP = 10,
        E_CONTROLLER_DIGITAL_DOWN = 11,
        E_CONTROLLER_DIGITAL_LEFT = 12,
        E_CONTROLLER_DIGITAL_RIGHT = 13,
        E_CONTROLLER_DIGITAL_X = 1,
        E_CONTROLLER_DIGITAL_A = 2,
        E_CONTROLLER_DIGITAL_B = 3,
        E_CONTROLLER_DIGITAL_Y = 4
    };
    
    enum controller_analog_e {
        E_CONTROLLER_ANALOG_LEFT_X = 1,
        E_CONTROLLER_ANALOG_LEFT_Y = 2,
        E_CONTROLLER_ANALOG_RIGHT_X = 3,
        E_CONTROLLER_ANALOG_RIGHT_Y = 4
    };
    
    enum controller_id_e {
        E_CONTROLLER_MASTER = 0
    };
    
    // Mock Motor class
    class Motor {
    private:
        int8_t m_port;
        double m_position = 0;
        double m_velocity = 0;
        double m_voltage = 0;
        v5::MotorGears m_gearset = v5::MotorGears::blue;
        v5::MotorBrake m_brakeMode = v5::MotorBrake::coast;
        
    public:
        Motor(int8_t port, v5::MotorGears gearset = v5::MotorGears::blue) 
            : m_port(port), m_gearset(gearset) {}
        
        std::int32_t move(std::int32_t voltage) {
            m_voltage = voltage;
            m_velocity = voltage * 6.0; // Mock velocity calculation
            return 1;
        }
        
        std::int32_t move_velocity(std::int32_t velocity) {
            m_velocity = velocity;
            m_voltage = velocity / 6.0;
            return 1;
        }
        
        double get_position() const { return m_position; }
        double get_actual_velocity() const { return m_velocity; }
        double get_voltage() const { return m_voltage; }
        
        void set_brake_mode(v5::MotorBrake mode) { m_brakeMode = mode; }
        v5::MotorBrake get_brake_mode() const { return m_brakeMode; }
        
        void reset_position() { m_position = 0; }
        void tare_position() { m_position = 0; }
        
        // Simulate movement
        void update(double dt) {
            m_position += m_velocity * dt / 60.0; // Convert RPM to degrees per second
        }
    };
    
    // Mock MotorGroup
    class MotorGroup {
    private:
        std::vector<int8_t> m_ports;
        std::vector<Motor> m_motors;
        
    public:
        MotorGroup(std::initializer_list<int8_t> ports, 
                   v5::MotorGears gearset = v5::MotorGears::blue) 
            : m_ports(ports.begin(), ports.end()) {
            for (int8_t port : m_ports) {
                m_motors.emplace_back(port, gearset);
            }
        }
        
        std::int32_t move(std::int32_t voltage) {
            for (auto& motor : m_motors) {
                motor.move(voltage);
            }
            return 1;
        }
        
        std::int32_t move_velocity(std::int32_t velocity) {
            for (auto& motor : m_motors) {
                motor.move_velocity(velocity);
            }
            return 1;
        }
        
        void set_brake_mode_all(v5::MotorBrake mode) {
            for (auto& motor : m_motors) {
                motor.set_brake_mode(mode);
            }
        }
        
        void update(double dt) {
            for (auto& motor : m_motors) {
                motor.update(dt);
            }
        }
    };
    
    // Mock IMU
    class IMU {
    private:
        double m_heading = 0;
        bool m_calibrating = false;
        
    public:
        IMU(uint8_t port) {}
        
        void reset() {
            m_heading = 0;
            m_calibrating = true;
        }
        
        bool is_calibrating() const { return m_calibrating; }
        
        void set_heading(double heading) {
            m_heading = heading;
            m_calibrating = false;
        }
        
        double get_heading() const { return m_heading; }
        
        void simulate_rotation(double degrees) {
            m_heading += degrees;
            while (m_heading >= 360) m_heading -= 360;
            while (m_heading < 0) m_heading += 360;
        }
    };
    
    // Mock Rotation sensor
    class Rotation {
    private:
        double m_position = 0; // in centidegrees
        
    public:
        Rotation(uint8_t port, bool reversed = false) {}
        
        void reset_position() { m_position = 0; }
        
        double get_position() const { return m_position; }
        
        void simulate_rotation(double degrees) {
            m_position += degrees * 100.0; // Convert to centidegrees
        }
    };
    
    // Mock Optical sensor
    class Optical {
    private:
        uint32_t m_hue = 0;
        uint32_t m_rgb = 0;
        
    public:
        Optical(uint8_t port) {}
        
        void set_led_pwm(uint8_t pwm) {}
        
        uint32_t get_hue() const { return m_hue; }
        uint32_t get_rgb() const { return m_rgb; }
        
        void set_color(uint32_t hue, uint32_t rgb) {
            m_hue = hue;
            m_rgb = rgb;
        }
    };
    
    // Mock ADI DigitalOut
    namespace adi {
        class DigitalOut {
        private:
            bool m_value = false;
            
        public:
            DigitalOut(char port) {}
            
            void set_value(bool value) { m_value = value; }
            bool get_value() const { return m_value; }
        };
    }
    
    // Mock Controller
    class Controller {
    private:
        std::map<int, bool> m_digital;
        std::map<int, int32_t> m_analog;
        std::map<int, bool> m_lastDigital;
        
    public:
        Controller(controller_id_e id) {
            // Initialize all buttons to false
            for (int i = 1; i <= 13; i++) {
                m_digital[i] = false;
                m_lastDigital[i] = false;
            }
            // Initialize analog sticks to 0
            for (int i = 1; i <= 4; i++) {
                m_analog[i] = 0;
            }
        }
        
        bool get_digital(controller_digital_e button) const {
            return m_digital.at(static_cast<int>(button));
        }
        
        bool get_digital_new_press(controller_digital_e button) {
            int btn = static_cast<int>(button);
            bool current = m_digital[btn];
            bool last = m_lastDigital[btn];
            m_lastDigital[btn] = current;
            return current && !last;
        }
        
        int32_t get_analog(controller_analog_e channel) const {
            return m_analog.at(static_cast<int>(channel));
        }
        
        void set_text(int line, int col, const char* text) {
            printf("[Controller Line %d]: %s\n", line, text);
        }
        
        void clear() {
            printf("[Controller]: Cleared\n");
        }
        
        void rumble(const char* pattern) {
            printf("[Controller]: Rumble %s\n", pattern);
        }
        
        // Test helpers
        void press_button(controller_digital_e button) {
            m_digital[static_cast<int>(button)] = true;
        }
        
        void release_button(controller_digital_e button) {
            m_digital[static_cast<int>(button)] = false;
        }
        
        void set_analog(controller_analog_e channel, int32_t value) {
            m_analog[static_cast<int>(channel)] = value;
        }
    };
    
    // Mock LCD
    namespace lcd {
        void initialize() {}
        void clear() {}
        void set_text(int line, const char* text) {
            printf("[LCD Line %d]: %s\n", line, text);
        }
    }
    
    // Mock battery
    namespace battery {
        double get_voltage() { return 12000; } // 12V in millivolts
        double get_current() { return 2000; }   // 2A in milliamps
        double get_capacity() { return 85; }    // 85%
    }
    
    // Mock delay
    void delay(uint32_t milliseconds) {
        // In real code, this would block
        // In tests, we can simulate time passing
    }
    
    // Mock Task
    class Task {
    public:
        Task(void (*task)(void*), void* param, const char* name) {}
        void remove() {}
    };
    
    // Mock Mutex
    class Mutex {
    private:
        bool m_locked = false;
        
    public:
        bool take(uint32_t timeout) {
            m_locked = true;
            return true;
        }
        
        bool give() {
            m_locked = false;
            return true;
        }
    };
    
    // Constants
    constexpr uint32_t TIMEOUT_MAX = UINT32_MAX;
}

#endif // MOCK_PROS_HPP

