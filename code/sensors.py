from machine import ADC, Pin

class Sensors:
    def __init__(self, positions_to_mux_channel, select_pins, adc_pin, alpha=0.9):
        """
        Initializes the sensors using an analog multiplexer.
        * positions_to_mux_channel: Mapping of position float -> mux channel (0-7)
        * select_pins: GPIO pins connected to S0, S1, S2 of the mux
        * adc_pin: ADC pin connected to the multiplexer output (e.g., ADC0)
        * alpha: smoothing factor for exponential moving average
        """
        self.positions_to_mux_channel = positions_to_mux_channel
        self.alpha = alpha
        self.prev_sensor_values = {pos: 0.0 for pos in positions_to_mux_channel}

        # Setup select pins
        self.select_lines = [Pin(pin, Pin.OUT) for pin in select_pins]

        # Setup ADC
        self.adc = ADC(Pin(adc_pin))
        
    def select_channel(self, channel):
        """
        Sets the multiplexer select lines to select the given channel (0-7)
        """
        for i, pin in enumerate(self.select_lines):
            pin.value((channel >> i) & 1)    

    def read_sensors(self):
        """
        Reads all sensors via the multiplexer with exponential smoothing.
        Returns a dictionary: position -> smoothed voltage.
        """
        sensor_voltages = {}

        for position, channel in self.positions_to_mux_channel.items():
            self.select_channel(channel)
            raw_adc = self.adc.read_u16()  # 16-bit ADC read
            voltage = (raw_adc / 65535.0) * 5  # Convert to voltage assuming 3.3V reference

            # Apply exponential smoothing
            smoothed = self.alpha * voltage + (1 - self.alpha) * self.prev_sensor_values[position]
            sensor_voltages[position] = smoothed
            self.prev_sensor_values[position] = smoothed

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