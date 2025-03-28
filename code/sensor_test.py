import time
from pimoroni_i2c import PimoroniI2C
from breakout_ioexpander import BreakoutIOExpander
from machine import I2C, Pin

PINS_BREAKOUT_GARDEN = {"sda": 12, "scl": 13}

ioe_adc_pins = [10, 12]
front_sensors = {"left": 10, "right": 12}

i2c = PimoroniI2C(**PINS_BREAKOUT_GARDEN)
ioe = BreakoutIOExpander(i2c, address=0x18)

for ioe_adc_pin in ioe_adc_pins:
    ioe.set_mode(ioe_adc_pin, BreakoutIOExpander.PIN_ADC)

while True:
    voltage_left = ioe.input_as_voltage(front_sensors["left"])
    voltage_right = ioe.input_as_voltage(front_sensors["right"])
    print(str(voltage_left) + " " + str(voltage_right))

    time.sleep(0.02)