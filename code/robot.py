from time import sleep
from pimoroni_i2c import PimoroniI2C
from breakout_ioexpander import BreakoutIOExpander
from array import array 
from machine import Pin, PWM

STOP = 0
RIGHTRIGHT = 1
RIGHT = 2
STRAIGHT = 3
LEFT = 4
LEFTLEFT = 5
NONE = 6

# Motor control pins
m1 = PWM(Pin(20))
m2 = PWM(Pin(21))
m3 = PWM(Pin(18))
m4 = PWM(Pin(19))

# Enable pins (if needed, set to output and enable)
en1 = Pin(17, Pin.OUT)
en2 = Pin(16, Pin.OUT)

# PWM frequency (adjust as needed)
PWM_FREQ = 1000  # Hz

m1.freq(PWM_FREQ)
m2.freq(PWM_FREQ)
m3.freq(PWM_FREQ)
m4.freq(PWM_FREQ)

def enable_motor():
    en1.value(1)  # motor 1 enable
    en2.value(1)  # motor 2 enable

def motor1_forward(speed):
    m1.duty_u16(int(speed * 655.35))  # 0-100% to 0-65535
    m2.duty_u16(0)

def motor1_reverse(speed):
    m1.duty_u16(0)
    m2.duty_u16(int(speed * 655.35))

def motor2_forward(speed):
    m3.duty_u16(int(speed * 655.35))
    m4.duty_u16(0)

def motor2_reverse(speed):
    m3.duty_u16(0)
    m4.duty_u16(int(speed * 655.35))

def motor_stop():
    for m in [m1, m2, m3, m4]:
        m.duty_u16(0)


def set_motors(direction):
    if direction == STRAIGHT:
        # all sensors on line - go straight
        motor1_forward(max_speed)
        motor2_forward(max_speed)
        #print('going straight')
    elif direction == LEFT:
        # left detection - gentle left turn
        motor1_forward(max_speed)
        motor2_forward(lesser_speed)
        #print('left turn')
    elif direction == RIGHT:
        # right detection - gentle right turn
        motor1_forward(lesser_speed)
        motor2_forward(max_speed)
        #print('right turn')
    elif direction == LEFTLEFT:
        # far left detection - sharp left turn
        motor1_forward(100)
        motor2_reverse(max_speed)
    elif direction == RIGHTRIGHT:
        # far right detection - sharp right turn
        motor1_reverse(max_speed)
        motor2_forward(100)
    elif direction == STOP:
        motor1_forward(0)
        motor2_forward(0)

THRESHOLD = 0.5

if __name__ == '__main__':
    # set pins

    PINS_BREAKOUT_GARDEN = {"sda": 12, "scl": 13}

    sensor_thresholds = {1: 0.5, 2: 2, 3: 0.5, 4: 0.5, 5: 0.5}
    sensor_positions = {1: "rightright", 2: "right", 3: "center", 4: "left", 5: "leftleft"}
    positions_sensors = {value: key for key, value in sensor_positions.items()}
    sensor_pins = {1: 10, 2: 7, 3: 9, 4: 11, 5: 13}
    ioe_adc_pins = sensor_pins.values()

    i2c = PimoroniI2C(**PINS_BREAKOUT_GARDEN)
    ioe = BreakoutIOExpander(i2c, address=0x18)

    for ioe_adc_pin in ioe_adc_pins:
        ioe.set_mode(ioe_adc_pin, BreakoutIOExpander.PIN_ADC)

    # initialize motor control
    enable_motor()
    
    max_speed = 50
    lesser_speed = 30

    memory_length = 50
    last_not_none = 0
    while True:
        # read sensors
        sensor_voltages = {}
        for sensor in sensor_pins.keys():
            sensor_voltages[sensor] = ioe.input_as_voltage(sensor_pins[sensor])
        
        voltage_left = sensor_voltages[positions_sensors["left"]]
        is_on_line_left = voltage_left > sensor_thresholds[positions_sensors["left"]]
        voltage_right = sensor_voltages[positions_sensors["right"]]
        is_on_line_right = voltage_right > sensor_thresholds[positions_sensors["right"]]
        voltage_center = sensor_voltages[positions_sensors["center"]]
        voltage_leftleft = sensor_voltages[positions_sensors["leftleft"]]
        voltage_rightright = sensor_voltages[positions_sensors["rightright"]]
        direction = STRAIGHT
        # P controller using all four sensors
#         if (voltage_center > sensor_thresholds[positions_sensors["center"]]):
            # all sensors on line - go straight
#             direction = "straight"
            #print('going straight')
        if (voltage_left > sensor_thresholds[positions_sensors["left"]] and voltage_right > sensor_thresholds[positions_sensors["right"]]):
            direction = STRAIGHT
            last_not_none = 0
        elif voltage_left > sensor_thresholds[positions_sensors["left"]]:
#         if voltage_left > sensor_thresholds[positions_sensors["left"]]:
            # left detection - gentle left turn
            direction = LEFT
            last_not_none = 0
            #print('left turn')
        elif voltage_right > sensor_thresholds[positions_sensors["right"]]:
            # right detection - gentle right turn
            direction = RIGHT
            last_not_none = 0
            #print('right turn')
        elif voltage_leftleft > sensor_thresholds[positions_sensors["leftleft"]]:
            # far left detection - sharp left turn
            direction = LEFTLEFT
            last_not_none = 0
        elif voltage_rightright > sensor_thresholds[positions_sensors["rightright"]]:
            # far right detection - sharp right turn
            direction = RIGHTRIGHT
            last_not_none = 0
        elif last_not_none >= memory_length:
            # no line detected - stop
            direction = STOP
            #print('not on line')
        set_motors(direction)

        print("leftleft=" + str(voltage_leftleft) + " left=" + str(voltage_left) + " center=" + str(voltage_center) + " right=" + str(voltage_right) + " rightright=" + str(voltage_rightright))
#         print(direction)
        sleep(0.02)



