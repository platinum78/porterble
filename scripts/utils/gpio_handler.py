#!/usr/bin/python3

"""
********************************************************************************
* Filename      : GPIO Handler
* Author        : Susung Park
* Description   : GPIO handler for communication with Arduino
* Version       : Initial release; 07 Jul 2019
********************************************************************************
"""

import RPi.GPIO as gpio

class GPIOHandler:
    INPUT = 1
    OUTPUT = 2
    def __init__(self, config_dict, simulation=False, debug=False):
        self.simulation = simulation
        self.debug = debug
        self.operating_mode = config_dict["operating_mode"]
        self.pins_dict = {}

        # Read pin configurations from dict.
        for pin in config_dict["pin_mode"]:
            if config_dict["pin_mode"][pin] == "OUTPUT":
                pins_dict[int(pin)] = self.OUTPUT
            elif config_dict["pin_mode"][pin] == "INPUT":
                pins_dict[int(pin)] = self.INPUT
            else:
                raise ValueError("Pin mode is improperly set.")
        
        # Interact with physical elements, if not simulation mode.
        if not self.simulation:
            import RPi.GPIO as gpio
            if self.operating_mode == "BCM":
                gpio.setmode(gpio.BCM)
            # Set the pins as given in the configuration dict.
            for pin in pins_dict.keys():
                if pins_dict[pin] == self.OUTPUT:
                    gpio.setup(pin_num, gpio.OUT)
                elif pins_dict[pin] == self.INPUT:
                    gpio.setup(pin_num, gpio.IN)
                
    def write_high(self, pin):
        if pin not in self.pins_dict.keys():
            raise RuntimeError("Pin [%d] is not set as output.")
        if not self.simulation:
            gpio.set(pin, gpio.HIGH)
    
    def write_low(self, pin):
        if pin not in self.pins_dict.keys():
            raise RuntimeError("Pin [%d] is not set as output.")
        if not self.simulation:
            gpio.set(pin, gpio.HIGH)
    
    def read(self, pin):
        if pin not in self.pins_dict.keys():
            raise RuntimeError("Pin [%d] is not set as input.")
        if not self.simulation:
            pass
