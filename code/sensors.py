from machine import ADC, Pin

class Sensors:
    """
    Analog sensor readings, storing and smoothing of readings.
    Conventions: 
        - both raw and smoothed readings are rescaled to the 0-5 range;
        - voltages are stored in lists, order of voltages is order of sensors from left to right.

    Attributes:
        positions_to_mux_channel (dict): Maps sensor positions to MUX channels.
        select_lines (list[Pin]): GPIO pins used to select MUX channel.
        adc (ADC): ADC object for reading sensor values.
        alpha (float): Smoothing factor for averaging over past readings.
        memory_length (int): Number of past readings used in smoothing.
        threshold_min (float): Voltages below this are treated as no line.
        threshold max (float): Voltages above this are treated as line fully detected.
        prev_voltages (list[list[float]]): History buffer of raw voltage readings.
        smoothed_prev_sensor_values (list[float]): Truncated and averaged voltages from the history buffer.
    """


    def __init__(self, positions_to_mux_channel, select_pins, adc_pin, alpha=0.9, memory_length=5, threshold_min = 0.8, threshold_max=3.4):
        """
        Initializes the sensors using an analog multiplexer.
        * positions_to_mux_channel: Mapping of position float -> mux channel (0-7)
        * select_pins: GPIO pins connected to S0, S1, S2 of the mux
        * adc_pin: ADC pin connected to the multiplexer output (e.g., ADC0)
        * alpha: smoothing factor
        * memory_length: number of previous sensor readings to store in memory
        * threshold_min: voltages below this are treated as no line detected
        * threshold max: voltages above this are treated as line fully detected
        """
        self.positions_to_mux_channel = positions_to_mux_channel
        self.alpha = alpha
        self.memory_length = memory_length
        self.threshold_min = threshold_min
        self.threshold_max = threshold_max

        #self.prev_sensor_values = {pos: 0.0 for pos in positions_to_mux_channel}

        self.select_lines = [Pin(pin, Pin.OUT) for pin in select_pins]

        self.adc = ADC(Pin(adc_pin))

        self.voltages = [0.0] * 7  # Initialize voltages for each sensor position
        
    def select_channel(self, channel):
        """
        Sets the multiplexer select lines to select the given channel (0-7)
        """
        for i, pin in enumerate(self.select_lines):
            pin.value((channel >> i) & 1)    

    def read_sensors(self):
        """
        Reads all sensors via the multiplexer.
        Returns a list of smoothed voltages.
        """

        # read voltages

        for position, channel in self.positions_to_mux_channel.items():
            self.select_channel(channel)
            raw_adc = self.adc.read_u16()  # 16-bit ADC read
            voltage = (raw_adc / 65535.0) * 5  # Convert to voltage assuming 5V reference
            self.voltages[int(position)-1] = voltage  # Store the voltage in the correct position


        # calculate and update the average of past readings
        self.get_truncated_and_smoothed_voltages()
        # self.smoothed_prev_sensor_values = smoothed_voltages

        return self.voltages
       
    def get_position_weighted_average(self):
        """
        Calculates the voltage weighted average of the sensor positions.
        Since the line is seen for high voltages, the average is calculated
        using the sensor voltages as the weights of their positions.
        * sensor_voltages: A dictionary of sensor voltages or a list of sensor voltages
        * voltages_in_list: you know what that is
        """
        total_weight = 0.0
        weighted_sum = 0.0
                
        for i, voltage in enumerate(self.voltages):

            total_weight += voltage

            weighted_sum += float(i + 1) * voltage

        return weighted_sum / total_weight if total_weight != 0 else 0
    
    def truncate(self, voltage):
        """
        Clips voltage value between self.threshold_min and self.threshold_max.
        Rescales the values to the 0-5 range.
        """
        val = min(self.threshold_max, voltage) if voltage > self.threshold_min else self.threshold_min
        
        val_01 = (val-self.threshold_min) / (self.threshold_max-self.threshold_min) 
        
        return val_01*5
    
    def get_truncated_and_smoothed_voltages(self):
        """
        Uses last n=self.memory_length sensor readings to calculate
        smoothed voltage value for each sensor. Before smoothing, clips
        voltage values between self.threshold_min and self.threshold_max.
        """
        
        for i in range(len(self.voltages)):
            self.voltages[i] = self.truncate(self.voltages[i])


    def get_current_line_position(self):
        """
        Calculates position of the line as average of sensor positions
        weighted by their smoothed n=self.memory_length last readings.
        Updates the value of self.smoothed_prev_sensor_values.
        """
        return self.get_position_weighted_average()