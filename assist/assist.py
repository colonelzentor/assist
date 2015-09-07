from __future__ import division, print_function
from warnings import warn
from numpy import exp, sqrt


def speed_of_sound(temperature):
    return sqrt(1.4 * 1716.56 * temperature)


def atmosphere(altitude, rho_sl=0.002378, temp_sl=59):
    """
    Density and Temperature as a function of altitude

    :param rho_sl: density at sea level (slugs/ft**3)
    :param temp_sl: temperature at sea level (degrees Fahrenheit)

    :type rho_sl: float
    :type temp_sl: float

    :rtype: tuple (density, temperature, speed of sound) (slugs/ft**3, degrees Rankine, ft/sec)

    """

    temp = temp_sl + 459.67

    if altitude < 36089:
        temperature = temp * (1 - altitude / 145442)
        return (rho_sl * (1 - altitude / 145442) ** 4.255876,
                temperature, speed_of_sound(temperature))
    elif altitude < 65617:
        temperature = temp * 0.751865
        return (rho_sl * 0.297076* exp((36089 - altitude) / 20806),
                temperature, speed_of_sound(temperature))
    elif altitude < 104987:
        temperature = temp * (0.682457 + altitude / 945374)
        return (rho_sl * (0.978261 + altitude / 659515) ** -35.16319,
                temperature, speed_of_sound(temperature))
    else:
        raise ValueError("Altitude of {:.1f} is too high, maximum altitude allowed is 104,987 ft.".format(altitude))


class Wing(object):
    """

    :param flap_type: type of flap on wing (plain, single_slot)
    :param sweep: quarter-chord sweep line (in degrees)

    """
    # List of default parameters [min, max, default value]
    _DEFAULTS = dict(sweep=[0, 60, 0, 'degrees'],
                     aspect_ratio=[1, 8, 4, 'unitless'],
                     flap_type=[None, None, 'none', 'n/a'],
                     configuration=[None, None, 'takeoff', 'n/a'],
                     taper_ratio = [0, 1, 1, 'unitless'],
                     flap_span = [0, 1, [0.3, 0.6], 'unitless'],
                     k_aero = [0, 1, 0.5, 'unitless'])

    _CL_MAX = {'none':           {'takeoff': [0.9, 1.2], 'landing': [0.9, 1.2]},
               'plain':          {'takeoff': [1.4, 1.6], 'landing': [1.7, 2.0]},
               'single_slot':    {'takeoff': [1.5, 1.7], 'landing': [1.8, 2.2]},
               'fowler':         {'takeoff': [2.0, 2.2], 'landing': [2.5, 2.9]},
               'double_slotted': {'takeoff': [1.7, 2.0], 'landing': [2.3, 2.7]},
               'triple_slotted': {'takeoff': [1.8, 2.1], 'landing': [2.7, 3.0]}}

    _SLAT_CL_DELTA = {'takeoff': 0.6, 'landing': 0.5}

    def _check_value(name, value, min_value=None, max_value=None, units='unitless'):
        if (min_value is not None and value < min_value) or (max_value is not None and val > max_value):
            raise ValueError("Value for '{}' [{} ({})] outside of bounds [{}, {}]".format(name, value, units, min_value, max_value))

    def __init__(self, **kwargs):
        for k, v in self.DEFAULTS.items():
            val = kwargs.pop(k, v[2])
            if hasattr(val, '__iter__'):
                for item in val:
                    _check_value(k, item, v[0], v[1], v[3])
            else:
                _check_value(k, val, v[0], v[1], v[3])
            setattr(self, k, val)

        if len(kwargs) > 0:
            warn("Unused arguments: {}".format(kwargs.keys()))

        self._reset()

    def _reset(self):
        """
        Resets cached variables.

        """
        self._cl_max = {}

    @property
    def takeoff(self):
        self.configuration = 'takeoff'
        return self

    @property
    def landing(self):
        self.configuration = 'landing'
        return self

    @property
    def cl_max(self):
        if self.configuration in self._cl_max:
            return self._cl_max[self.configuration]
        else:
            self._cl_max[self.configuration] = self._estimate_cl_max()

    def _estimate_cl_max(self):
        """
        Estimates the maximum coefficient of lift (CL_max) for a wing given the flap
        and slat configuration.

        Based on Raymer, 1999 (pp. 97) and Mattingly, 2002 (pp. 36).

        .. assumptions::
            - wing is unswept or swept back
            - wing is thin (<)
            - aspect ratio is less than 8

        """
        # Based on table in Mattingly, 2002 (pp. 36)

        configuration = self.configuration
        sweep = self.sweep
        taper_ratio = self.taper_ratio
        flap_type = self.flap_type
        flap_span = self.flap_span
        CL_MAX = self._CL_MAX
        k_aero = self.k_aero

        if self.aspect_ratio > 8:
            warn("Estimates not valid for high aspect ratio wings, ideally AR " +
                 "should be less than 8, you specified {}".format(aspect_ratio))

        if sweep > 60:
            warn("Estimates not valid for sweeps > 60 degrees, "+
                 "you specified {}".format(sweep))

        if flap_type not in CL_MAX:
            raise ValueError("flap_type '{}' specified does not correspond with the options available {}.".format(flap_type, CL_MAX.keys()))

        cl = CL_MAX['none'][configuration]
        cl_max_unflapped = k_aero * (cl[1] - cl[0]) + cl[0]

        cl = CL_MAX[flap_type][configuration]
        cl_max_flapped = k_aero * (cl[1] - cl[0]) + cl[0]

        if self.slats:
            cl = self.SLAT_CL_DELTA[configuration]
            cl_max_unflapped += cl
            cl_max_flapped += cl

        s_ratio = (2 + (taper_ratio - 1) * sum(flap_span)) * \
                  (flap_span[1] - flap_span[0]) / (1 + taper_ratio)

        # Regressed from Fig. 5.3 in Raymer, 1999 (pp. 97)
        sweep_factor = 2 - (0.00011029411764705700 * sweep * sweep + 0.00014705882352927800 * sweep + 1.00294117647059000000)

        return sweep_factor * (0.85 + 0.1 * k_aero) * (cl_max_flapped * s_ratio + cl_max_unflapped * (1 - s_ratio))
