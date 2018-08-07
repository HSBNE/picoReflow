#!/usr/bin/python
import logging

from w1thermsensor import W1ThermSensor

class MAX31850(object):
    def __init__(self, w1id):
        self.max31850 = W1ThermSensor(W1ThermSensor.THERM_SENSOR_THERM_SENSOR_MAX31850K, w1id)
        self.log = logging.getLogger(__name__)

    def get(self):
        '''Reads w1thermo thermocouple.'''
        degC = self.max31850.get_temperature()
        '''self.log.debug("status %s" % state)'''
        return degC
