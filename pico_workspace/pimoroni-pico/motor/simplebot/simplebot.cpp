#include "motor2040.hpp"
#include "encoder.hpp"
#include "button.hpp"
#include "pid.hpp"
#include <cstdio>
#include <cstdlib>

#include <unistd.h>
#include <sys/select.h> // Include select() function

#include <cstring>
#include <unistd.h>
#include "pico/stdlib.h"
#include <string>

// Wheel friendly names
constexpr int C = 2;
constexpr int D = 3;
constexpr int B = 1;
constexpr int A = 0;

constexpr float GEAR_RATIO = 21.3;                // The gear ratio of the motors
constexpr float COUNTS_PER_REV = 44 * GEAR_RATIO; // The counts per revolution of each motor's output shaft

constexpr float SPEED_SCALE = 3.78; // The scaling to apply to each motor's speed to match its real-world speed

constexpr int UPDATES = 100; // How many times to update the motor per second
constexpr float UPDATE_RATE = 10.0 / UPDATES;
constexpr float ZERO_POINT = 0.0; // The duty cycle that corresponds with zero speed when plotting the motor's speed as a straight line
constexpr float DEAD_ZONE = 5;    // The duty cycle below which the motor's friction prevents it from moving

// PID values
float VEL_KP = 3;      // Velocity proportional (P) gain
float VEL_KI = 0.0;     // Velocity integral (I) gain
float VEL_KD = 0.002f; // Velocity derivative (D) gain

using namespace plasma;
using namespace motor;
using namespace encoder;

// Create a list of motors with a given speed scale
const pin_pair MOTOR_PINS[] = {motor2040::MOTOR_B, motor2040::MOTOR_C, motor2040::MOTOR_A, motor2040::MOTOR_D};
const uint NUM_MOTORS = sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);
Motor *motors[NUM_MOTORS];

// Create a list of encoders, using PIO 0, with the given counts per revolution
const pin_pair ENCODER_PINS[] = {motor2040::ENCODER_B, motor2040::ENCODER_C, motor2040::ENCODER_A, motor2040::ENCODER_D};
const char *ENCODER_NAMES[] = {"A", "B", "D", "C"};
const uint NUM_ENCODERS = sizeof(ENCODER_PINS) / sizeof(ENCODER_PINS[0]);
Encoder *encoders[NUM_ENCODERS];

const uint NUM_PARAMS = 3;
uint response_type = 0; // 0 for encoder 1 for params
// Create PID objects for position control
PID vel_pids[NUM_MOTORS];

// Create the user button
Button user_sw(motor2040::USER_SW);

class MySerial
{
public:
    MySerial()
    {
    }

    char *read_serial()
    {
        static char data[255];
        if (scanf("%255s", data) == 1)
        {
            return data;
        }
        return nullptr;
    }

    int send_serial(const char *data)
    {
        // Check if the data is null-terminated
        size_t len = strlen(data);
        if (len > 0 && data[len - 1] != '\0')
        {
            // Create a new buffer with an additional space for the null terminator
            char buffer[len + 1];
            strcpy(buffer, data); // Copy the original data
            buffer[len] = '\0';   // Append the null terminator

            // Send the null-terminated data
            printf("%s\n", buffer);
        }
        else
        {
            // Data is already null-terminated
            printf("%s\n", data);
        }
        return 1;
    }
};

void set_motor_speeds(Motor **motors, PID *vel_pids, float *speeds)
{
    for (uint i = 0; i < NUM_MOTORS; ++i)
    {
        vel_pids[i].setpoint = speeds[i];
    }
}


Encoder::Capture captures[NUM_MOTORS];
// Define your classes and functions here

void set_motors_pids(float kp, float ki, float kd) {
    for (auto i = 0u; i < NUM_MOTORS; i++) {
        // Delete previous instances if they exist
        if (motors[i] != nullptr) {
            delete motors[i];
        }
        if (encoders[i] != nullptr) {
            delete encoders[i];
        }

        // Create and initialize motors
        motors[i] = new Motor(MOTOR_PINS[i], NORMAL_DIR, SPEED_SCALE, 0, 0.1);
        motors[i]->init();

        // Create and initialize encoders
        encoders[i] = new Encoder(pio0, i, ENCODER_PINS[i], PIN_UNUSED, NORMAL_DIR, COUNTS_PER_REV, true);
        encoders[i]->init();

        // Initialize PID controllers with the provided constants
        vel_pids[i] = PID(kp, ki, kd, UPDATE_RATE);
    }
}


int main()
{
    printf("hello\n");
    stdio_init_all();
    while (!stdio_usb_connected())
    {
        sleep_ms(100);
    }
    // Create instances
    MySerial serial_obj;

    set_motors_pids(VEL_KP, VEL_KI, VEL_KD);

    // Enable all motors
    for (uint i = 0; i < NUM_MOTORS; ++i)
    {
        motors[i]->enable();
    }

    Encoder::Capture captures[NUM_ENCODERS];

    // Continually move the motor until the user button is pressed
    while (!user_sw.raw())
    {
        // sleep_ms(5000);
        //  Capture the state of all the encoders
        //  Read serial input for controlling motors

        char *serial_input = serial_obj.read_serial();
        if (serial_input != nullptr && !strncmp(serial_input, "VEL_KP", strlen("VEL_KP")) == 0 && !strncmp(serial_input, "VEL_KI", strlen("VEL_KI")) == 0 && !strncmp(serial_input, "VEL_KD", strlen("VEL_KD")) == 0)
        {
            response_type = 0;
            float speeds[NUM_MOTORS];
            char *token = strtok(serial_input, ",");
            uint idx = 0;
            while (token != nullptr && idx < NUM_MOTORS)
            {
                speeds[idx++] = std::atof(token);
                token = strtok(nullptr, ","); // nullptr means parsing the same string
            }
            set_motor_speeds(motors, vel_pids, speeds);
        }
        else if (serial_input != nullptr && strncmp(serial_input, "VEL_KP", strlen("VEL_KP")) == 0) {
            response_type = 1;
            char serial_copy[128];

            // Safely copy and ensure null-termination
            strncpy(serial_copy, serial_input, sizeof(serial_copy) - 1);
            serial_copy[sizeof(serial_copy) - 1] = '\0';
    
            char *token = serial_copy + strlen("VEL_KP:");
            VEL_KP = std::stof(std::string(token));

            // Reinitialize motors, encoders, and PIDs with new constants
            set_motors_pids(VEL_KP, VEL_KI, VEL_KD);
        }
        else if (serial_input != nullptr && strncmp(serial_input, "VEL_KI", strlen("VEL_KI")) == 0) {
            response_type = 1;
            char serial_copy[128];

            // Safely copy and ensure null-termination
            strncpy(serial_copy, serial_input, sizeof(serial_copy) - 1);
            serial_copy[sizeof(serial_copy) - 1] = '\0';
    
            char *token = serial_copy + strlen("VEL_KI:");
            VEL_KI = std::stof(std::string(token));

            // Reinitialize motors, encoders, and PIDs with new constants
            set_motors_pids(VEL_KP, VEL_KI, VEL_KD);
        }
        else if (serial_input != nullptr && strncmp(serial_input, "VEL_KD", strlen("VEL_KD")) == 0) {
            response_type = 1;
            char serial_copy[128];

            // Safely copy and ensure null-termination
            strncpy(serial_copy, serial_input, sizeof(serial_copy) - 1);
            serial_copy[sizeof(serial_copy) - 1] = '\0';
    
            char *token = serial_copy + strlen("VEL_KD:");
            VEL_KD = std::stof(std::string(token));

            // Reinitialize motors, encoders, and PIDs with new constants
            set_motors_pids(VEL_KP, VEL_KI, VEL_KD);
        }
        // Capture the state of all the encoders
        char output[200];
        int offset = 0; // Keeps track of the current position in the buffer
        //char rotation_per_second[256];
        for (uint i = 0; i < NUM_ENCODERS; ++i)
        {
            captures[i] = encoders[i]->capture();
            // printf("%f", captures[i].revolutions_per_second());
            offset += sprintf(output + offset, "%f ", captures[i].revolutions_per_second());
        }

        if (response_type){
            printf("PID: KP=%.4f, KI=%.4f, KD=%.4f\n", VEL_KP, VEL_KI, VEL_KD);
        }
        else{
            printf("%s\n", output);
        }
        
        //rotation_per_second[strlen(rotation_per_second) - 1] = '\0'; // Remove trailing comma

        // Send data over serial if needed

        for (uint i = 0; i < NUM_MOTORS; ++i)
        {
            
            // Calculate the acceleration to apply to the motor to move it closer to the velocity setpoint
            float accel = vel_pids[i].calculate(captures[i].revolutions_per_second());

            // Accelerate or decelerate the motor
            motors[i]->speed(motors[i]->speed() + (accel * UPDATE_RATE));
        }

        sleep_ms(UPDATE_RATE);
    }

    // Stop all the motors
    for (uint i = 0; i < NUM_MOTORS; ++i)
    {
        motors[i]->disable();
        delete motors[i];
    }

    // Delete encoders
    for (uint i = 0; i < NUM_ENCODERS; ++i)
    {
        delete encoders[i];
    }

    return 0;
}
