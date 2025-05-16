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


    def __init__(self, positions_to_mux_channel, select_pins, adc_pin, alpha=0.9, memory_length=5, threshold_min = 0.5, threshold_max=4.5):
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

        self.prev_voltages = [[0 for s in range(len(positions_to_mux_channel))] for i in range(memory_length)]

        self.smoothed_prev_sensor_values = self.get_truncated_and_smoothed_voltages()

        
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
        current_voltages = []

        for position, channel in self.positions_to_mux_channel.items():
            self.select_channel(channel)
            raw_adc = self.adc.read_u16()  # 16-bit ADC read
            voltage = (raw_adc / 65535.0) * 5  # Convert to voltage assuming 5V reference
            current_voltages.append(voltage)

        # update memory
        # ordering in memory list: frow newest to oldest
        updated_prev_voltages = self.prev_voltages[:-1]
        updated_prev_voltages.insert(0, current_voltages)
        self.prev_voltages = updated_prev_voltages

        # calculate and update the average of past readings
        smoothed_voltages = self.get_truncated_and_smoothed_voltages()

        return smoothed_voltages
       
    def get_position_weighted_average(self, sensor_voltages, voltages_in_list=False):
        """
        Calculates the voltage weighted average of the sensor positions.
        Since the line is seen for high voltages, the average is calculated
        using the sensor voltages as the weights of their positions.
        * sensor_voltages: A dictionary of sensor voltages or a list of sensor voltages
        * voltages_in_list: you know what that is
        """
        total_weight = 0.0
        weighted_sum = 0.0
                
        if voltages_in_list:
            sensor_voltages = {float(pos+1) : sensor_voltages[pos] for pos in range(len(sensor_voltages))}

        for position, voltage in sensor_voltages.items():

            total_weight += voltage

            weighted_sum += position * voltage

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
        after_threshold = [[self.truncate(val) for val in line] for line in self.prev_voltages]
        
        smoothed = after_threshold[0]

        for voltages in after_threshold[1:]:
            for i in range(len(smoothed)):
                smoothed[i] = self.alpha*smoothed[i] + (1-self.alpha)*voltages[i]
         
        return smoothed


    def get_current_line_position(self):
        """
        Calculates position of the line as average of sensor positions
        weighted by their smoothed n=self.memory_length last readings.
        Updates the value of self.smoothed_prev_sensor_values.
        """
        smoothed = self.get_truncated_and_smoothed_voltages()

        self.smoothed_prev_sensor_values = smoothed

        return self.get_position_weighted_average(smoothed, voltages_in_list=True)