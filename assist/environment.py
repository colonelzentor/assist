from __future__ import division
from math import exp, sqrt


G_0 = 32.2


class Atmosphere(object):
    """
    Atmospheric calculations.

    :param density_sl: density at sea level (slugs/ft**3)
    :param temp_sl: temperature at sea level (degrees Fahrenheit)

    :type density_sl: float
    :type temp_sl: float

    """

    def __init__(self, density_sl=0.002378, temperature_sl=59.0):
        self.density_sl = density_sl
        self.temperature_sl = temperature_sl

    @property
    def temperature_sl_rankine(self):
        return self.temperature_sl + 459.67

    def density(self, altitude):
        """
        Density as a function of altitude, in slugs/ft**3

        :param altitude: altitude in feet
        :type altitude: float

        :rtype: float

        """

        if altitude < 36089:
            return self.density_sl * (1 - altitude / 145442) ** 4.255876
        elif altitude < 65617:
            return density_sl * 0.297076* exp((36089 - altitude) / 20806)
        elif altitude <= 104987:
            return density_sl * (0.978261 + altitude / 659515) ** -35.16319
        raise ValueError("Altitude of {:.1f} is too high, maximum altitude allowed is 104,986 ft.".format(altitude))

    def temperature(self, altitude):
        """
        Temperature as a function of altitude, in degrees Rankine

        :param altitude: altitude in feet
        :type altitude: float

        :rtype: float

        """

        if altitude < 36089:
            return self.temperature_sl_rankine * (1 - altitude / 145442)
        elif altitude < 65617:
            return self.temperature_sl_rankine * 0.751865
        elif altitude <= 104987:
            return self.temperature_sl_rankine * (0.682457 + altitude / 945374)
        raise ValueError("Altitude of {:.1f} is too high, maximum altitude allowed is 104,986 ft.".format(altitude))

    def speed_of_sound(self, altitude):
        """
        Speed of Sound as a function of altitude, in ft/sec

        :param altitude: altitude in feet
        :type altitude: float

        :rtype: float

        """

        return sqrt(1.4 * 1716.56 * self.temperature(altitude))

    def specific_heat_ratio(self, altitude):
        temperature = self.temperature(altitude)
        t = (temperature - 419.67) / 1540

        if t < 0:
            return 1.40107995826834

        return 0.131099998803052 * t * t * t - 0.21091027609333 * t * t + 0.00781004072769065 * t + 1.40107995826834
