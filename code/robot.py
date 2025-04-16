from time import sleep
from pimoroni_i2c import PimoroniI2C
from breakout_ioexpander import BreakoutIOExpander
from array import array 
from machine import Pin, PWM

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



THRESHOLD = 0.5

if __name__ == '__main__':
    # set pins

    PINS_BREAKOUT_GARDEN = {"sda": 12, "scl": 13}

    sensor_thresholds = {1: 0.5, 2: 0.5, 3: 0.5, 4: 0.5, 5: 0.5}
    sensor_positions = {1: "rightright", 2: "center", 3: "right", 4: "left", 5: "leftleft"}
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
        # P controller using all four sensors
        if (voltage_center > THRESHOLD):
            # all sensors on line - go straight
            speed_l = max_speed
            speed_r = max_speed
            #print('going straight')
        elif voltage_left > THRESHOLD:
            # far left detection - sharp left turn
            speed_l = max_speed
            speed_r = lesser_speed
            #print('left turn')
        elif voltage_right > THRESHOLD:
            # far right detection - sharp right turn
            speed_l = lesser_speed
            speed_r = max_speed
            #print('right turn')
        elif voltage_leftleft > THRESHOLD:
        #     # left detection - gentle left turn
             speed_l = 100
             speed_r = lesser_speed
        elif voltage_rightright > THRESHOLD:
        #     # right detection - gentle right turn
             speed_l = lesser_speed
             speed_r = 100
        else:
            # no line detected - stop
            speed_l = 0
            speed_r = 0
            #print('not on line')
        #speed_l = 0#max_speed
        #speed_r = 0#max_speed
        motor1_forward(speed_l)
        motor2_forward(speed_r)
        #print("left=" + str(is_on_line_left) + " right=" + str(is_on_line_right))
        print("left=" + str(voltage_left) + " center=" + str(voltage_right) + " right=" + str(voltage_center) + " leftleft=" + str(voltage_leftleft) + " rightright=" + str(voltage_rightright))
        sleep(0.02)



