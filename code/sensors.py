from pimoroni_i2c import PimoroniI2C
from breakout_ioexpander import BreakoutIOExpander

PINS_BREAKOUT_GARDEN = {"sda": 12, "scl": 13}

class Sensors:
    def __init__(self, positions_to_pins, i2c_pins=PINS_BREAKOUT_GARDEN, alpha=0.9):
        """
        Initializes the sensors with the given positions and pins.
        * positions_to_pins: A dictionary mapping sensor positions to their corresponding pins.
        * i2c_pins: A dictionary containing the I2C SDA and SCL pins.
        * alpha: The smoothing factor for the exponential moving average.
        """
        # Pin positions setup
        self.positions_to_pins = positions_to_pins
        self.ioe_adc_pins = self.positions_to_pins.values()

        # Exponential moving average setup
        self.prev_sensor_values = {position: 0.0 for position in self.positions_to_pins.keys()}
        self.alpha = alpha

        # I2C setup
        i2c = PimoroniI2C(**i2c_pins)
        self.ioe = BreakoutIOExpander(i2c, address=0x18)
        for ioe_adc_pin in self.ioe_adc_pins:
            self.ioe.set_mode(ioe_adc_pin, BreakoutIOExpander.PIN_ADC)

    def read_sensors(self):
        """
        Reads the sensor values and applies exponential smoothing.
        Returns a dictionary of smoothed sensor values.
        """
        sensor_voltages = {}
        for position, pin in self.positions_to_pins.items():
            raw_value = self.ioe.input_as_voltage(pin)
            # Apply exponential smoothing
            smoothed_value = self.alpha * raw_value + (1 - self.alpha) * self.prev_sensor_values[position]
            sensor_voltages[position] = smoothed_value
            self.prev_sensor_values[position] = smoothed_value

        return sensor_voltages
    
    def get_position_weighted_average(self, sensor_voltages):
        """
        Calculates the voltage weighted average of the sensor positions.
        Since the line is seen for high voltages, the average is calculated
        using the sensor voltages as the weights of their positions.
        * sensor_voltages: A dictionary of sensor voltages.
        """
        total_weight = 0.0
        weighted_sum = 0.0

        for position, voltage in sensor_voltages.items():
            total_weight += voltage
            weighted_sum += position * voltage

        return weighted_sum / total_weight if total_weight != 0 else 0