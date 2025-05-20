from time import sleep
import time
from sensors import Sensors
from motors import Motors
from pid import PDController
from machine import Pin


DEBUG = False
MODE = 'line pos'
# MODE = 'sensor voltages'
# MODE='remote'

motor_pins = {
    "left_forward": 21,
    "left_reverse": 20,
    "right_forward": 18,
    "right_reverse": 19
}

motor_enable_pins = [17, 16]

# Sensor positions to pins mapping, with leftmost sensor at 1.0 and rightmost at 5.0
positions_to_mux_channel = {
    1.0: 1,
    2.0: 2,
    3.0: 3,
    4.0: 4,
    5.0: 0
}

select_pins = (27, 12, 13)
adc_pin = 28 
rc_pin = 26

THRESHOLD_MIN = .8
THRESHOLD_MAX = 3.4
EPSILON = 0.3   # safety threshold for line not visible (to avoind comparing values with 0)
EPSILON_UPPER = 4.

BASE_SPEED = 90  ### should rename to: max crusing speed
K_s = 0.2  # Scaling factor for speed adjustment
K_sd = 0.2
PROPORTION = 4.0 #proportion left vs right for tight turns


STOPPED = False
LEFT = True

tight=True
is_on = False


def debug(mode, new_is_on):
    start_time = time.ticks_ms()           
    sensors.read_sensors()
    if mode=='line pos':
        print(sensors.get_current_line_position())    
        
    if mode=='sensor voltages':
        sensor_voltages2 = sensors.get_truncated_and_smoothed_voltages()
        output = ''
        for sensor in range(1, 6):
            output += 's'+str(int(sensor)) + '=' + str(sensor_voltages2[sensor-1]) + ' '
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
    rc_PIN = Pin(rc_pin, Pin.IN)
    # Initialize motors and sensors
    motors = Motors(motor_pins, motor_enable_pins)
    motors.start()
    sensors = Sensors(positions_to_mux_channel, select_pins, adc_pin, threshold_min=THRESHOLD_MIN, memory_length=5, threshold_max=THRESHOLD_MAX, alpha=0.99)

    # Initialize PD controller
    dt = 0.01 # Time step in seconds
    Kp = -15.0  # Proportional gain
    Kd = -0.6  # Derivative gain
    pd_controller = PDController(Kp, Kd, dt, setpoint=3.0)
    
    try:
        if DEBUG:
            while True:
                is_on = debug(MODE, is_on)

        while True:
            print(is_on)
            # Measure how much time this loop takes by first getting the time
            if not rc_PIN.value():
                is_on = False if is_on else True
                sleep(0.3)
            if not is_on:
                if not STOPPED:
                    STOPPED = True
                    motors.stop()
                sleep(0.01)
                continue
            else:
                if STOPPED:
                    STOPPED = False
                    motors.start()
            start_time = time.ticks_ms()
            sensor_voltages = sensors.read_sensors()
            sensor_voltages_smoothed_list = sensors.smoothed_prev_sensor_values
            position_weighted_average = sensors.get_current_line_position()

            control_output, error, derivative = pd_controller.update(position_weighted_average)
            
#             if all(voltage > EPSILON_UPPER for voltage in sensor_voltages_smoothed_list[:-1]) or all(voltage > EPSILON_UPPER for voltage in sensor_voltages_smoothed_list[1:]):
#                 break

            # If all sensors are below the threshold, stop the motors
            if all(voltage < EPSILON for voltage in sensor_voltages_smoothed_list) or all(voltage > EPSILON_UPPER for voltage in sensor_voltages_smoothed_list[:-1]) or all(voltage > EPSILON_UPPER for voltage in sensor_voltages_smoothed_list[1:]):
#             LEFT = True
#             if True:
#                 motors.stop()
#                 motors.start()
                if LEFT:
                    motors.stop()
                    motors.start()
                    motors.tight_turn(-80.0, PROPORTION)
    
                else:
                    
                    motors.stop()
                    motors.start()
    
                    motors.tight_turn(80.0, PROPORTION)
#                 break
            else:
#                 if STOPPED:
#                     STOPPED = False
#                     motors.start()
                if position_weighted_average < 3.0:
                    LEFT = True
                else:
                    LEFT = False
                speed = BASE_SPEED / (1 + K_s * abs(error) + K_sd * abs(derivative))  #zmniejszyć stałe 
                motors.set_direction(speed, control_output)
            
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