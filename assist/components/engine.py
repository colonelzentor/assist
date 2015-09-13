from __future__ import division
from numpy import sqrt, exp
from util import verify_value
from environment import Atmosphere

__all__ = ('Engine',)

class Engine(object):
    _TYPES = dict(ATJ={
                      'name': 'Advanced Turbo-Jet (with afterburner)',
                      'alpha': [1, 1.00, 0.952, 0.30, 0.40, 2.0, 0.7],
                      'tsfc_coefficients': {'normal': [1.1, 0.30],
                                            'afterburner': [1.5, 0.23]},
                      'bpr': 0,
                  },
                  ATP={
                      'name': 'Advanced Turbo-Prop',
                      'alpha': [1, 0.12, 0.000, 1.00, -0.02, -1.0, 0.5],
                      'tsfc_coefficients': {'normal': [0.18, 0.80]},
                      'bpr': None,
                  },
                  HBTF={
                      'name': 'High By-Pass Turbo-Fan',
                      'alpha': [-1, 1.00, 0.568, 0.25, -1.20, 3.0, 0.6],
                      'tsfc_coefficients': {'normal': [0.45, 0.54]},
                      'bpr': 6,
                  },
                  LBTF={
                      'name': 'Low By-Pass Turbo-Fan (with afterburner)',
                      'alpha': [1, 1.00, 0.940, 0.38, 0.40, 2.0, 0.7],
                      'tsfc_coefficients': {'normal': [0.9, 0.30],
                                            'afterburner': [1.6, 0.27]},
                      'bpr': 1.5,
                  })

    def __init__(self,
                 engine_type='ATJ',
                 max_thrust=None,
                 max_mach=None,
                 afterburner=False,
                 k_sfc=1.0,
                 k_w=1.0,
                 k_size=1.0,
                 turbine_inlet_temp=2000,
                 atmosphere=None, *args, **kwargs):

        self.engine_type = engine_type
        self.afterburner = afterburner
        self.k_sfc = k_sfc
        self.k_w = k_w
        self.k_size = k_size
        self.atmosphere = Atmosphere() if atmosphere is None else atmosphere
        self.turbine_inlet_temp = turbine_inlet_temp

        if 'bpr' in kwargs:
            self.bpr = kwargs.pop('bpr')
            if self.bpr < 2.0 and engine_type == 'HBTF':
                raise ValueError(
                    "High By-Pass Turbo-Fans must have by-pass ratios greater than 2.0, not {}".format(
                        self.bpr))
            elif self.bpr > 2.0 and engine_type == 'LBTF':
                raise ValueError(
                    "Low By-Pass Turbo-Fans must have by-pass ratios less than 2.0, not {}".format(
                        self.bpr))
        else:
            self.bpr = self._TYPES[engine_type]['bpr']

        self.max_mach = max_mach
        self.max_thrust = max_thrust

        if engine_type not in self._TYPES:
            raise ValueError(
                "Engine type must be one of these {}, not {}".format(
                    self._TYPES.keys(), configuration))

        self.alphas = self._TYPES[engine_type]['alpha']
        self.name = self._TYPES[engine_type]['name']
        self._tfsc_coefficients = self._TYPES[engine_type]['tsfc_coefficients']

    def __repr__(self):
        return "<Engine {}>".format(self.name)

    def tsfc(self, mach, altitude, afterburner=False):
        """
        Estimates TSFC as a function of Mach number and altitude.

        .. note::
            based on Mattingly, 2002 (pp. 71)

        :param mach: Mach number at which engine is flying
        :param altitude: altitude at which engine is flying
        :param afterburner: whether or not afterburners are engaged

        :type mach: float
        :type altitude: float
        :type afterburner: bool

        :rtype: float

        """
        a, b = self._tfsc_coefficients[
            'afterburner'
        ] if afterburner else self._tfsc_coefficients['normal']

        theta = self.atmosphere.temperature(
            altitude=altitude) / self.atmosphere.temperature_sl_rankine

        return (a + b * mach) * sqrt(theta)

    def size(self):
        """
        Determines a jet engine's parameters based on Raymer's rules as defined in:

        Raymer, D. P., "Aircraft design: a conceptual approach", 3rd Ed., pp. 235

        .. note::
            Cruise is assumed to be at approximately 36,000 ft (11,000 m) and 0.9 Mach

        """

        if self.afterburner:
            if self.bpr > 1.0:
                raise (NotImplementedError(
                    "BPR must be less than 1.0 for afterburning engines, bpr = {}".format(
                        self.bpr)))
            self.w = 0.063 * (self.max_thrust ** 1.1) * \
                (self.max_mach ** 0.25) * exp(-0.81 * self.bpr)
            self.l = 0.255 * (self.max_thrust ** 0.4) * (self.max_mach ** 0.2)
            self.d = 0.024 * (self.max_thrust ** 0.5) * exp(0.04 * self.bpr)
            self.sfc_max = 2.1 * exp(-0.12 * self.bpr)
            self.t_cruise = 2.4 * (self.max_thrust ** 0.74
                                   ) * exp(0.023 * self.bpr)
            self.sfc_cruise = 1.04 * exp(-0.186 * self.bpr)
        else:
            self.w = 0.084 * (self.max_thrust ** 1.1) * exp(-0.045 * self.bpr)
            self.l = 0.185 * (self.max_thrust ** 0.4) * (self.max_mach ** 0.2)
            self.d = 0.033 * (self.max_thrust ** 0.5) * exp(0.04 * self.bpr)
            self.sfc_max = 0.67 * exp(-0.12 * self.bpr)
            self.t_cruise = 0.60 * \
                (self.max_thrust ** 0.9) * exp(0.02 * self.bpr)
            self.sfc_cruise = 0.88 * exp(-0.05 * self.bpr)

    def thrust_lapse(self, altitude, mach=None, speed=None):
        """
        Calculates the reduction in thrust for the engine as a function of altitude and speed.

        :param altitude: altitude at which the engine is flying
        :param mach: Mach number (optional)
        :param speed: speed at which the aircraft is flying (must be specified if mach is None)

        :type altitude: float
        :type mach: float
        :type speed: float

        :rtype: float

        """

        sign, a1, a2, a3, a4, a5, a6 = self.alphas

        if mach is None:
            if speed is None:
                raise ValueError(
                    "Must specify Mach number or speed (in ft/sec)")
            mach = speed / self.atmosphere.speed_of_sound(altitude)

        density_ratio = self.atmosphere.density(
            altitude) / self.atmosphere.density_sl

        return a1 * (a2 + a3 * (sign * mach - a4) ** a5) * density_ratio ** a6
