from __future__ import division
from math import sqrt
from util import verify_value
from environment import Atmosphere


class Wing(object):
    """

    :param flap_type: type of flap on wing (plain, single_slot)
    :param sweep: quarter-chord sweep line (in degrees)

    """
    _AR_VS_MACH = {
        'jet_trainer': [4.737, -0.979],
        'jet_fighter': [5.416, -0.622],
        'jet_attack': [4.110, -0.622],
        'mil_cargo': [5.570, -1.075],
        'bomber': [5.570, -1.075],
        'transport': [7.500, 0.000]
    }

    # List of default parameters [min, max, default value]
    _DEFAULTS = dict(sweep=[0, 60, 0, 'degrees', '_sweep'],
                     aspect_ratio=[1, 8, 4, 'unitless'],
                     flap_type=[None, None, 'none', 'n/a'],
                     slats=[None, None, False, 'n/a'],
                     configuration=[None, None, 'takeoff', 'n/a'],
                     taper_ratio=[0, 1, 1, 'unitless'],
                     flap_span=[0, 1, [0.3, 0.6], 'unitless'],
                     k_aero=[0, 1, 0.5, 'unitless'])

    _CL_MAX = {
        'none': {'takeoff': [0.9, 1.2],
                 'landing': [0.9, 1.2]},
        'plain': {'takeoff': [1.4, 1.6],
                  'landing': [1.7, 2.0]},
        'single_slot': {'takeoff': [1.5, 1.7],
                        'landing': [1.8, 2.2]},
        'fowler': {'takeoff': [2.0, 2.2],
                   'landing': [2.5, 2.9]},
        'double_slotted': {'takeoff': [1.7, 2.0],
                           'landing': [2.3, 2.7]},
        'triple_slotted': {'takeoff': [1.8, 2.1],
                           'landing': [2.7, 3.0]}
    }

    _SLAT_CL_DELTA = {'takeoff': 0.6, 'landing': 0.5}

    def __init__(self, **kwargs):
        for k, v in self._DEFAULTS.items():
            val = kwargs.pop(k, v[2])
            if hasattr(val, '__iter__'):
                for item in val:
                    verify_value(k, item, v[0], v[1], v[3])
            else:
                verify_value(k, val, v[0], v[1], v[3])
            if len(v) > 4:
                k = v[4]
            setattr(self, k, val)

        if not hasattr(
                self,
                'ar') and 'aircraft_type' in kwargs and 'design_mach' in kwargs:
            aircraft_type = kwargs.pop('aircraft_type')
            mach = kwargs.pop('design_mach')
            if aircraft_type not in self._AR_VS_MACH:
                raise ValueError(
                    "Aircraft Type of {} is not one of the allowed values {}".format(
                        aircraft_type, self._AR_VS_MACH.keys()))
            a, c = self._AR_VS_MACH[aircraft_type]
            setattr(self, 'ar', a * mach ** c)

        if len(kwargs) > 0:
            warn("Unused arguments: {}".format(kwargs.keys()))

        self._reset()

    def __repr__(self):
        high_lift = "No Flaps" if self.flap_type == 'none' else self.flap_type
        if self.slats:
            high_lift += ' Slats'
        return "<Wing (High Lift Devices {}, Configured for {})>".format(
            high_lift, self.configuration)

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
    def cruise(self):
        self.configuration = 'cruise'
        return self

    @property
    def sweep(self):
        if hasattr(self._sweep, '__iter__'):
            raise NotImplementedError(
                "Variable sweep functionality has not been implemented yet")
        else:
            return self._sweep

    @property
    def cl_max(self):
        if self.configuration in self._cl_max:
            return self._cl_max[self.configuration]
        else:
            self._cl_max[self.configuration] = self._estimate_cl_max()
            return self._cl_max[self.configuration]

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
            warn(
                "Estimates not valid for high aspect ratio wings, ideally AR "
                +
                "should be less than 8, you specified {}".format(aspect_ratio))

        if sweep > 60:
            warn("Estimates not valid for sweeps > 60 degrees, " +
                 "you specified {}".format(sweep))

        if flap_type not in CL_MAX:
            raise ValueError(
                "flap_type '{}' specified does not correspond with the options available {}.".format(
                    flap_type, CL_MAX.keys()))

        cl = CL_MAX['none'][configuration]
        cl_max_unflapped = k_aero * (cl[1] - cl[0]) + cl[0]

        cl = CL_MAX[flap_type][configuration]
        cl_max_flapped = k_aero * (cl[1] - cl[0]) + cl[0]

        if self.slats:
            cl = self._SLAT_CL_DELTA[configuration]
            cl_max_unflapped += cl
            cl_max_flapped += cl

        s_ratio = (2 + (taper_ratio - 1) * sum(flap_span)) * \
                  (flap_span[1] - flap_span[0]) / (1 + taper_ratio)

        # Regressed from Fig. 5.3 in Raymer, 1999 (pp. 97)
        sweep_factor = 2 - (0.00011029411764705700 * sweep * sweep +
                            0.00014705882352927800 * sweep +
                            1.00294117647059000000)

        return sweep_factor * (0.85 + 0.1 * k_aero) * (
            cl_max_flapped * s_ratio + cl_max_unflapped * (1 - s_ratio))


class Engine(object):
    _TYPES = dict(ATJ={
        'name': 'Advanced Turbo-Jet (with afterburner)',
        'alpha': [1, 1.00, 0.952, 0.30, 0.40, 2.0, 0.7],
        'tsfc_coefficients':
        {'normal': [1.1, 0.30],
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
                      'tsfc_coefficients':
                      {'normal': [0.9, 0.30],
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


class Payload(object):
    def __init__(self, name=None,
                 weight=0.0,
                 cd_r=0.0,
                 expendable=False, *args, **kwargs):
        self.name = name
        self.weight = weight
        self.cd_r = cd_r
        self.expendable = expendable

    def __repr__(self):
        return "<Payload {} ({} lbm)>".format(self.name, self.weight)
