from time import sleep
import time
from sensors import Sensors
from motors import Motors
from pid import PIDController
from buffer import CyclicBuffer
import _thread
from machine import Pin, time_pulse_us

DEBUG = False
# MODE = 'line pos'
MODE = 'sensor voltages'
# MODE='remote'

# Pins for the motors, multiplexer and remote control
motor_pins = {
    "left_forward": 21,
    "left_reverse": 20,
    "right_forward": 18,
    "right_reverse": 19
}
motor_enable_pins = [17, 16]

select_pins = (27, 12, 13)
adc_pin = 28 

rc_pin = 26

# Mapping of sensor position to multiplexer channel
# leftmost sensor is at 1.0 and rightmost is at 7.0
positions_to_mux_channel = {
    1.0: 5,
    2.0: 0,
    3.0: 1,
    4.0: 2,
    5.0: 3,
    6.0: 4,
    7.0: 6,
}

positions_to_mux_channel = [5, 0, 1, 2, 3, 4, 6]

# Sensor constants
THRESHOLD_MIN = 3.4      # Minimun voltage for sensor input thresholding
THRESHOLD_MAX = 5.0     # Maximum voltage for sensor input thresholding
EPSILON = 0.5           # Lower sensor inputs are considered to not be on the line
EPSILON_UPPER = 4.      # Higher sensor inputs are considered to be on the line
N_smoothing_memory = 2  # Length of memory buffer for sensor smoothing
smoothing_alpha = 0.99  # Alpha parameter for sensor smoothing
# border_mode = 'last'
border_mode = 'average'
N_direction_memory = 10

# Motor speed constants
K_se = 0.15              # Speed scaling for error values
K_sd = 0.15             # Speed scaling for derivative values
TIGHT_TURN_SPEED = 125.0 # The speed for tight turns
BASE_SPEED = 100.0         # The speed at which the robot moves when the line is perfectly aligned
PROPORTION = 1.5        # Proportion of wheel speeds for tight turns

# PID constants
middle_of_line = 4.0    # Point we consider to be the desired position
dt = 0.01               # Time step in seconds
Kp = -25.0               # Proportional gain
Kd = -0.4               # Derivative gain
Ki = -0.0              # Integral gain

battery_constant = 0.8
# 8.86 - 0.47
# 7.54 - 1.0


# Shared pulse signal and result
shared = {"new_pulse": False, "pulse_len": 0}
lock = _thread.allocate_lock()

def pulse_reader(rc_PIN, lock, shared):
    while True:
        # Read the pulse duration (blocking)
        pulse = time_pulse_us(rc_PIN, 1, 1000000)
        if pulse > 0:
            with lock:
                if shared["new_pulse"] == False:
                    shared["pulse_len"] = pulse
                    shared["new_pulse"] = True
        sleep(0.01)


def debug(mode, new_is_on, sensors):
    start_time = time.ticks_ms()           
    sensors.read_sensors()
    print(sensors.voltages)
    if mode=='line pos':
        print(sensors.get_current_line_position())    
        
    if mode=='sensor voltages':
        output = ''
        for sensor in range(7):
            output += 's'+str(int(sensor)) + '=' + str(sensors.voltages[sensor]) + ' '
        print(output)
    end_time = time.ticks_ms()
    elapsed_time_ms = time.ticks_diff(end_time, start_time)
    elapsed_time_s = elapsed_time_ms / 1000.0
    if elapsed_time_s < dt:
        # Sleep for the remaining time to maintain the loop frequency
        sleep_time = dt - elapsed_time_s
        sleep(sleep_time)
    if mode=='remote':
        rc_PIN = Pin(rc_pin, Pin.IN)
#         is_on = is_on
#         print(rc_PIN.value())
        if not rc_PIN.value():
            new_is_on = False if new_is_on else True
            sleep(0.3)
        print(new_is_on)
        return new_is_on
    return False


if __name__ == "__main__":

    # Initialize remote control
    rc_PIN = Pin(rc_pin, Pin.IN)
    #_thread.start_new_thread(pulse_reader, [rc_PIN, lock, shared])

    # Initialize motors, sensors
    motors = Motors(motor_pins, motor_enable_pins)
    motors.start()
    sensors = Sensors(positions_to_mux_channel, select_pins, adc_pin, threshold_min=THRESHOLD_MIN, memory_length=N_smoothing_memory, threshold_max=THRESHOLD_MAX, alpha=smoothing_alpha)

    # Initialize turn detection variables
    direction_buffer = CyclicBuffer(5)
    left_sensor_readings = CyclicBuffer(N_direction_memory)
    right_sensor_readings = CyclicBuffer(N_direction_memory)
    steps_line_right = 0       # Number of time steps since line was read by leftmost sensor
    steps_line_left = 0        # Number of time steps since line was read by rightmost sensor

    # Initialize PID controller
    pid_controller = PIDController(Kp, Kd, Ki, dt, setpoint=middle_of_line)

    # Robot state flags
    after_sharp_turn = False   # Tells if we just got out of a sharp turn
    stopped = False            # Tells if the motors are stopped  
    left = True                # Tells which direction we should turn if we lose the line next step
    is_on = False              # Tells if the robot is active (on/off from remote)
        
    try:
        if DEBUG:
            while True:
                is_on = debug(MODE, is_on, sensors)

        while True:
            with lock:
                if shared["new_pulse"]:
                    print("Pulse length:", shared["pulse_len"], "us")
                    if shared["pulse_len"] > 50000:
                        is_on = False if is_on else True
                    shared["pulse_len"] = 0
                    shared["new_pulse"] = False
            # if not rc_PIN.value():
            #     is_on = False if is_on else True
            #     sleep(0.3)
            is_on = True
            if not is_on:
                if not stopped:
                    stopped = True
                    motors.stop()
                    # Reset the buffer after being turned off
                    direction_buffer = CyclicBuffer(N_direction_memory)
                sleep(dt)
                continue
            else:
                if stopped:
                    stopped = False
                    motors.start()

            # Measure start time for equal timesteps
            start_time = time.ticks_ms()

            sensor_voltages = sensors.read_sensors()
            position_weighted_average = sensors.get_current_line_position()

            control_output, error, derivative = pid_controller.update(position_weighted_average)
            speed = BASE_SPEED / (1 + K_se * abs(error) + K_sd * abs(derivative))
            
            # If all sensors do not see the line or all see the line, make a tight turn to get back on the line
            all_off_line = all(voltage < EPSILON for voltage in sensors.voltages)
            all_on_line  = all(voltage > EPSILON_UPPER for voltage in sensors.voltages)
            if all_off_line or all_on_line:
                # A neutral update to the PID controller and error buffer
                pid_controller.update(middle_of_line)
                direction_buffer.append(0.0)
                # Update flag that we were executing a sharp turn this time step
                after_sharp_turn = True
                motors.stop()
                motors.start()
                if left:
                    motors.tight_turn(-battery_constant * TIGHT_TURN_SPEED, PROPORTION)
                else:
                    motors.tight_turn(battery_constant * TIGHT_TURN_SPEED, PROPORTION)    
            else:

                # Stop the motors and try to stop spinning after ending the sharp turn
                if after_sharp_turn:
                    motors.stop()
                    after_sharp_turn = False
                    sleep(dt)
                    motors.start()
                    continue

                if border_mode == 'last':

                    if sensors.voltages[0] > EPSILON_UPPER:
                        steps_line_left = 0
                    else:
                        steps_line_left += 1

                    if sensors.voltages[6] > EPSILON_UPPER:
                        steps_line_right = 0
                    else:
                        steps_line_right += 1

                    if steps_line_left < steps_line_right:
                        left = True
                    else:
                        left = False

                if border_mode == 'average':

                    left_sensor_readings.append(max(sensors.voltages[0], 0.0))
                    right_sensor_readings.append(max(sensors.voltages[6], 0.0))
                    
                    print('l=' + str(left_sensor_readings.average() + 1.0) + ' r=' + str(right_sensor_readings.average() + 1.0)) 

                    if left_sensor_readings.average() > right_sensor_readings.average():
                        left = True
                    else:
                        left = False
                
                motors.set_direction(battery_constant * speed, battery_constant * control_output)
                direction_buffer.append(error)
            
            end_time = time.ticks_ms()
            elapsed_time_ms = time.ticks_diff(end_time, start_time)
            elapsed_time_s = elapsed_time_ms / 1000.0
            if elapsed_time_s < dt:
                # Sleep for the remaining time to maintain the loop frequency
                sleep_time = dt - elapsed_time_s
                sleep(sleep_time)
                
    except KeyboardInterrupt:
        pass
    finally:
        motors.stop()
