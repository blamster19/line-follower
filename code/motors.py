from machine import Pin, PWM

class Motors:
    def __init__(self, motor_pins, enable_pins, pwm_freq=1000):
        """
        Initializes the motors with the given pins and frequency.
        * motor_pins: A dictionary mapping motor names to their corresponding pins.
        * enable_pins: A list of enable pins for each motor.
        * pwm_freq: The PWM frequency for the motors.
        """
        self.motors = {name: PWM(Pin(pin)) for name, pin in motor_pins.items()}
        self.enable_pins = [Pin(pin, Pin.OUT) for pin in enable_pins] if enable_pins else None
        self.pwm_freq = pwm_freq

        for motor in self.motors.values():
            motor.freq(self.pwm_freq)

    def set_direction(self, base_speed, direction):
        
        right_motor_speed = max(min(base_speed - direction, 100.0), -100.0)
        left_motor_speed = max(min(base_speed + direction, 100.0), -100.0)

        if (right_motor_speed > 0):
            self.motors["right_forward"].duty_u16(int(right_motor_speed * 655.35))
            self.motors["right_reverse"].duty_u16(0)
        else:
            self.motors["right_forward"].duty_u16(0)
            self.motors["right_reverse"].duty_u16(int(-right_motor_speed * 655.35))

        if (left_motor_speed > 0):
            self.motors["left_forward"].duty_u16(int(left_motor_speed * 655.35))
            self.motors["left_reverse"].duty_u16(0)
        else:
            self.motors["left_forward"].duty_u16(0)
            self.motors["left_reverse"].duty_u16(int(-left_motor_speed * 655.35))
            
    def tight_turn(self, direction):
        
        if direction >= 0.0:
            left_motor_speed = (direction / 4.0)
            right_motor_speed = -direction
        else:
            left_motor_speed = direction
            right_motor_speed = -direction / 4.0
            
        if (right_motor_speed > 0):
            self.motors["right_forward"].duty_u16(int(right_motor_speed * 655.35))
            self.motors["right_reverse"].duty_u16(0)
        else:
            self.motors["right_forward"].duty_u16(0)
            self.motors["right_reverse"].duty_u16(int(-right_motor_speed * 655.35))

        if (left_motor_speed > 0):
            self.motors["left_forward"].duty_u16(int(left_motor_speed * 655.35))
            self.motors["left_reverse"].duty_u16(0)
        else:
            self.motors["left_forward"].duty_u16(0)
            self.motors["left_reverse"].duty_u16(int(-left_motor_speed * 655.35))
            

    def stop(self):
        for motor in self.motors.values():
            motor.duty_u16(0)
        if self.enable_pins:
            for enable_pin in self.enable_pins:
                enable_pin.value(0)

    def start(self):
        if self.enable_pins:
            for enable_pin in self.enable_pins:
                enable_pin.value(1)

        