from picozero import Pin, pico_led
from time import sleep
from motor import Motor, pico_motor_shim
from pimoroni import NORMAL_DIR, REVERSED_DIR
from array import array 

if __name__ == '__main__':
    # set pins

    ## initialize sensors
    sensors = []
    sensors.append(Pin(12, Pin.IN))
    sensors.append(Pin(13, Pin.IN))
    sensors.append(Pin(14, Pin.IN))
    sensors.append(Pin(15, Pin.IN))
    sensors.append(Pin(16, Pin.IN))
    sensors.append(Pin(17, Pin.IN))
    sensors.append(Pin(18, Pin.IN))
    sensors.append(Pin(19, Pin.IN))

    ## initialize motors
    SPEED_SCALE = 5.4
    DRIVING_SPEED = SPEED_SCALE
    motor_left = Motor(pico_motor_shim.MOTOR_1, direction=NORMAL_DIR, speed_scale=SPEED_SCALE)
    motor_right = Motor(pico_motor_shim.MOTOR_2, direction=REVERSED_DIR, speed_scale=SPEED_SCALE)

    # define data

    sensor_number = 8
    sensor_history_len = 4
    sensor_history = array('i',[0]*sensor_number)
    speed_l = 0
    speed_r = 0
    history_step = 0

    # define helper functions

    def read_sensors(history_index):
        for i in range(sensor_number):
            if sensors[i].value():
                sensor_history[history_index] |= (1 << i)
            else:
                sensor_history[history_index] &= ~(1 << i)

    def set_motors(left_speed, right_speed):
        motor_left.speed(left_speed)
        motor_right.speed(right_speed)

    def stop():
        motor_left.stop()
        motor_right.stop()

    def coast():
        motor_left.coast()
        motor_right.coast()

    def calculate_speeds(history_index):
        pass

    # main loop
    
    while True:
        read_sensors(history_step)
        speed_l, speed_r = calculate_speeds(history_step)
        set_motors(speed_l, speed_r)        
        history_step = (history_step + 1) % sensor_history_len
