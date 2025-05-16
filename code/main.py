from time import sleep
import time
from sensors import Sensors
from motors import Motors
from pid import PDController

motor_pins = {
    "left_forward": 21,
    "left_reverse": 20,
    "right_forward": 18,
    "right_reverse": 19
}

motor_enable_pins = [17, 16]

# Sensor positions to pins mapping, with leftmost sensor at 1.0 and rightmost at 5.0
positions_to_mux_channel = {
    1.0: 0,
    2.0: 1,
    3.0: 2,
    4.0: 3,
    5.0: 4
}

select_pins = (27, 12, 13)
adc_pin = 28 

THRESHOLD_MIN = .8 
THRESHOLD_MAX = 3.4
EPSILON = 0.8   # safety threshold for line not visible (to avoind comparing values with 0)

BASE_SPEED = 90
K_s = 0.2  # Scaling factor for speed adjustment
K_sd = 1.0
STOPPED = False
LEFT = True

if __name__ == "__main__":
    # Initialize motors and sensors
    motors = Motors(motor_pins, motor_enable_pins)
    motors.start()
    sensors = Sensors(positions_to_mux_channel, select_pins, adc_pin, threshold_min=THRESHOLD_MIN, memory_length=5, threshold_max=THRESHOLD_MAX, alpha=0.99)

    # Initialize PD controller
    dt = 0.01 # Time step in seconds
    Kp = -15.0  # Proportional gain
    Kd = -1  # Derivative gain
    pd_controller = PDController(Kp, Kd, dt, setpoint=3.0)
    
#     try:
#         while True:
#         # Measure how much time this loop takes by first getting the time
#             start_time = time.ticks_ms()
#             sensor_voltages = sensors.read_sensors()
#             sensor_voltages2 = sensors.get_truncated_and_smoothed_voltages()
# 
#             output = ''
# #            for sensor in sensor_voltages.keys():
# #                output += 's'+str(int(sensor)) + '=' + str(sensor_voltages[sensor]) + ' '
#             for sensor in range(1, 6):
#                  output += 's'+str(int(sensor)) + '=' + str(sensor_voltages2[sensor-1]) + ' '
#             print(output)
            #print(sensors.get_current_line_position())    
    try:
        while True:
            # Measure how much time this loop takes by first getting the time
            start_time = time.ticks_ms()
            sensor_voltages = sensors.read_sensors()
#             output = ''
#             for sensor in sensor_voltages.keys():
#                output += 's'+str(int(sensor)) + '=' + str(sensor_voltages[sensor]) + ' '
#             print(output)

            # If all sensors are below the threshold, stop the motors
            if all(voltage < EPSILON for voltage in sensor_voltages.values()) or all(voltage > THRESHOLD_MAX for voltage in sensor_voltages.values()):
#             LEFT = True
#             if True:
                if LEFT:
                    motors.tight_turn(-80.0)
                else:
                    motors.tight_turn(80.0)
            else:
                if STOPPED:
                    STOPPED = False
                    motors.start()
                position_weighted_average = sensors.get_current_line_position()
                if position_weighted_average < 3.0:
                    LEFT = True
                else:
                    LEFT = False
                control_output, error, derivative = pd_controller.update(position_weighted_average)
                speed = BASE_SPEED / (1 + K_s * (abs(error) + K_sd * abs(derivative)))
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