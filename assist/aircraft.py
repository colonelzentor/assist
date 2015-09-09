from __future__ import division, print_function
from warnings import warn

from numpy import array, pi, exp, sqrt, log, max, argmin, cos, sin, abs
from scipy.interpolate import interp1d

from util import verify_value
from environment import Atmosphere


class Aircraft(object):
    """
    Conceptual-level Aircraft definition.

    :param sweep: sweep of the wing at the quarter-chord
    :type sweep: list, tuple, float

    """

    # TODO: get a better way to estimate the T/W initially
    _T_TO_W = {'jet_trainer': 0.75,
               'jet_fighter': 1.20,
               'mil_cargo':   0.45,
               'bomber':      0.65,
               'jet_transport': 0.35}

    _DESIGN_MACH = {'jet_trainer': 0.95,
                    'jet_fighter': 1.50,
                    'mil_cargo':   0.85,
                    'bomber':      0.92,
                    'jet_transport': 0.78}

    _CD_0 = {'jet_fighter': {'mach': [  0.0,   0.8,    0.9,    1.1,   1.2,   2.0],
                             'min':  [0.014, 0.014, 0.0160, 0.0260, 0.028, 0.028],
                             'max':  [0.018, 0.018, 0.0235, 0.0345, 0.040, 0.038]}}

    _K_1 = {'jet_fighter': {'mach':  [  0.0,   0.8,   1.0,   1.2,   2.0],
                            'min':   [0.180, 0.180, 0.180, 0.216, 0.360],
                            'max':   [0.140, 0.140, 0.170, 0.200, 0.500]}}

    _CONFIGURATIONS = ('takeoff', 'landing')

    def __init__(self,
                 aircraft_type='jet_fighter',
                 configuration=None,
                 wing=None,
                 engine='ATJ',
                 num_engines=1,
                 stores=None,
                 k_aero=0.5,
                 k_2=0.003,
                 k_to=1.1,
                 *args, **kwargs):

        self.type = aircraft_type
        self.configuration = configuration
        self.design_mach = self._DESIGN_MACH[aircraft_type]
        self.wing = Wing(k_aero=k_aero,
                         aircraft_type=aircraft_type,
                         design_mach=self.design_mach) if not isinstance(wing, Wing) else wing
        self.engine = Engine(engine) if isinstance(engine, basestring) else engine
        self.num_engines = num_engines

        # TODO: implement variable sweep analysis
        self.variable_sweep = False

        self.t_to_w = self._T_TO_W[aircraft_type]

        self.stores = [] if stores is None else stores
        self._stores = self.stores[:]

        self.k_aero = k_aero
        self._cd_0 = kwargs.pop('cd_0', None)
        self._cl_max = kwargs.pop('cl_max', None)

        self._k_1 = kwargs.pop('k_1', None)
        self.k_2 = k_2

        if self._cd_0 is None and aircraft_type in self._CD_0:
            self._cd_0_min = interp1d(self._CD_0[aircraft_type]['mach'],
                                      self._CD_0[aircraft_type]['min'],
                                      bounds_error=False, fill_value=0.02)
            self._cd_0_max = interp1d(self._CD_0[aircraft_type]['mach'],
                                      self._CD_0[aircraft_type]['max'],
                                      bounds_error=False, fill_value=0.02)
        else:
            self._cd_0 = 0.02

        if self._k_1 is None and aircraft_type in self._K_1:
            self._k_1_min = interp1d(self._K_1[aircraft_type]['mach'],
                                     self._K_1[aircraft_type]['min'],
                                     bounds_error=False, fill_value=0.02)
            self._k_1_max = interp1d(self._K_1[aircraft_type]['mach'],
                                     self._K_1[aircraft_type]['max'],
                                     bounds_error=False, fill_value=0.02)
        else:
            self._k_1 = 0.16

        self.k_to = k_to

        self.cl_max = self.wing.cl_max
        self.takeoff = self.wing.takeoff
        self.landing = self.wing.landing
        self.thrust_lapse = self.engine.thrust_lapse

    @property
    def payload(self):
        total = 0
        for store in self.stores:
            total += store.weight
        return total

    def __repr__(self):
        return "<Aircraft {} ({}, {})>".format(self.type, str(self.wing), str(self.engine))

    def cd_r(self, stores=None):
        stores = self.stores[:] if stores is None else stores
        cd_r = 0
        for store in stores:
            # TODO add Store class with functionality to calculate its own drag
            cd_r += 0.02

    @property
    def cd_0(self):
        if self._cd_0 is not None:
            return self._cd_0
        else:
            if getattr(self, 'mach', None) is None:
                raise AttributeError("Must set the mach number")
            min_cd_0 = self._cd_0_min(self.mach)
            max_cd_0 = self._cd_0_max(self.mach)
            return min_cd_0 + (max_cd_0 - min_cd_0) * (1 - self.k_aero)

    @property
    def k_1(self):
        if self._k_1 is not None:
            return self._k_1
        else:
            if getattr(self, 'mach', None) is None:
                raise AttributeError("Must set the mach number")
            min_k_1 = self._k_1_min(self.mach)
            max_k_1 = self._k_1_max(self.mach)
            return min_k_1 + (max_k_1 - min_k_1) * (1 - self.k_aero)

    def synthesize(self, segments, wing_loading=None):
        """
        Identifies a design point for a mission

        """

        wing_loadings = array(range(10, 300))
        thrust_loadings = []
        weight_fraction = 1.0

        self.max_mach = 0
        for segment in segments:
            if segment.mach > self.max_mach:
                self.max_mach = segment.mach
                self.mach = segment.mach
            thrust_loadings.append(segment.thrust_to_weight_required(
                                       aircraft=self,
                                       wing_loading=wing_loadings,
                                       prior_weight_fraction=weight_fraction
                                       )
                                  )
            weight_fraction *= segment.weight_fraction
            print("Segment {} has a weight fraction of {}".format(segment.name,
                                                                  segment.weight_fraction))

        self.fuel_fraction = 1 - weight_fraction

        self.t_to_w_req = max(array(zip(*thrust_loadings)), 1)
        idx = argmin(self.t_to_w_req)

        self.t_to_w = self.t_to_w_req[idx]
        self.w_to_s = wing_loadings[idx]

    def size(self, segments, w_to=(1000, 60000), tol=10):
        """
        Sizes the aircraft for a given mission

        Empty Weight Fraction (we_to_w0) based on (Raymer, 1999) pp. 115

        """

        if hasattr(w_to, '__iter__'):
            w_to = array(range(w_to[0], w_to[1], tol))

        # Empty weight coefficients
        coefficients = {'jet_trainer':   ( 0.00, 4.28, -0.10, 0.10, 0.20, -0.24, 0.11),
                        'jet_fighter':   (-0.02, 2.16, -0.10, 0.20, 0.04, -0.10, 0.08),
                        'mil_cargo':     ( 0.07, 1.71, -0.10, 0.10, 0.06, -0.10, 0.05),
                        'bomber':        ( 0.07, 1.71, -0.10, 0.10, 0.06, -0.10, 0.05),
                        'jet_transport': ( 0.32, 0.66, -0.13, 0.30, 0.06, -0.05, 0.05)}

        if self.type not in coefficients:
            raise NotImplementedError("Aircraft type '{}' not implemented, " +
                                      "only these have been implemented: {}".format(self.type, coefficients.keys()))

        for segment in segments:
            pass
        wf_to_w0 = 1 - segments[-1].prior_weight_fraction * segments[-1].weight_fraction

        a, b, c1, c2, c3, c4, c5 = coefficients[self.type]
        k_vs = 1.04 if self.variable_sweep else 1.0

        we_to_w0 = (a + b * w_to ** c1 * \
                            self.wing.aspect_ratio ** c2 * \
                            self.t_to_w ** c3 * \
                            self.w_to_s ** c4 * \
                            self.max_mach ** c5) * k_vs

        w_to_calc = self.payload / (wf_to_w0 - we_to_w0)

        idx = argmin(abs(w_to_calc - w_to))

        self.w_to = w_to_calc[idx]

        self.engine.max_mach = self.max_mach
        self.engine.max_thrust = self.t_to_w * self.w_to / self.num_engines
        self.wing.area = self.w_to / self.w_to_s


class Wing(object):
    """

    :param flap_type: type of flap on wing (plain, single_slot)
    :param sweep: quarter-chord sweep line (in degrees)

    """
    _AR_VS_MACH = {'jet_trainer': [4.737, -0.979],
                   'jet_fighter': [5.416, -0.622],
                   'jet_attack':  [4.110, -0.622],
                   'mil_cargo':   [5.570, -1.075],
                   'bomber':      [5.570, -1.075],
                   'transport':   [7.500,  0.000]}

    # List of default parameters [min, max, default value]
    _DEFAULTS = dict(sweep=[0, 60, 0, 'degrees', '_sweep'],
                     aspect_ratio=[1, 8, 4, 'unitless'],
                     flap_type=[None, None, 'none', 'n/a'],
                     slats=[None, None, False, 'n/a'],
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

        if not hasattr(self, 'ar') and 'aircraft_type' in kwargs and 'design_mach' in kwargs:
            aircraft_type = kwargs.pop('aircraft_type')
            mach = kwargs.pop('design_mach')
            if aircraft_type not in self._AR_VS_MACH:
                raise ValueError("Aircraft Type of {} is not one of the allowed values {}".format(aircraft_type, self._AR_VS_MACH.keys()))
            a, c = self._AR_VS_MACH[aircraft_type]
            setattr(self, 'ar', a * mach ** c)

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
    def sweep(self):
        if hasattr(self._sweep, '__iter__'):
            raise NotImplementedError("Variable sweep functionality has not been implemented yet")
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
            cl = self._SLAT_CL_DELTA[configuration]
            cl_max_unflapped += cl
            cl_max_flapped += cl

        s_ratio = (2 + (taper_ratio - 1) * sum(flap_span)) * \
                  (flap_span[1] - flap_span[0]) / (1 + taper_ratio)

        # Regressed from Fig. 5.3 in Raymer, 1999 (pp. 97)
        sweep_factor = 2 - (0.00011029411764705700 * sweep * sweep + 0.00014705882352927800 * sweep + 1.00294117647059000000)

        return sweep_factor * (0.85 + 0.1 * k_aero) * (cl_max_flapped * s_ratio + cl_max_unflapped * (1 - s_ratio))


class Engine(object):
    _TYPES = dict(ATJ= {'name': 'Advanced Turbo-Jet (with afterburner)',
                        'alpha': [ 1, 1.00, 0.952, 0.30,  0.40,  2.0, 0.7],
                        'tsfc_coefficients': {'normal': [1.1, 0.30],
                                              'afterburner': [1.5, 0.23]},
                        'bpr': 0,
                        },
                  ATP= {'name': 'Advanced Turbo-Prop',
                        'alpha': [ 1, 0.12, 0.000, 1.00, -0.02, -1.0, 0.5],
                        'tsfc_coefficients': {'normal': [0.18, 0.80]},
                        'bpr': None,
                        },
                  HBTF={'name': 'High By-Pass Turbo-Fan',
                        'alpha': [-1, 1.00, 0.568, 0.25, -1.20,  3.0, 0.6],
                        'tsfc_coefficients': {'normal': [0.45, 0.54]},
                        'bpr': 6,
                        },
                  LBTF={'name': 'Low By-Pass Turbo-Fan (with afterburner)',
                        'alpha': [ 1, 1.00, 0.940, 0.38,  0.40,  2.0, 0.7],
                        'tsfc_coefficients': {'normal': [0.9, 0.30],
                                              'afterburner': [1.6, 0.27]},
                        'bpr': 1.5,
                        })

    def __init__(self, engine_type='ATJ',
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
                raise ValueError("High By-Pass Turbo-Fans must have by-pass ratios greater than 2.0, not {}".format(self.bpr))
            elif self.bpr > 2.0 and engine_type == 'LBTF':
                raise ValueError("Low By-Pass Turbo-Fans must have by-pass ratios less than 2.0, not {}".format(self.bpr))
        else:
            self.bpr = self._TYPES[engine_type]['bpr']

        self.max_mach = max_mach
        self.max_thrust = max_thrust

        if engine_type not in self._TYPES:
            raise ValueError("Engine type must be one of these {}, not {}".format(self._TYPES.keys(), configuration))

        self.alphas = self._TYPES[engine_type]['alpha']
        self.name = self._TYPES[engine_type]['name']
        self._tfsc_coefficients = self._TYPES[engine_type]['tsfc_coefficients']

    def __repr__(self):
        return "<{}>".format(self.name)

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
        a, b = self._tfsc_coefficients['afterburner'] if afterburner else self._tfsc_coefficients['normal']

        theta = self.atmosphere.temperature(altitude=altitude) / self.atmosphere.temperature_sl_rankine

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
                raise(NotImplementedError("BPR must be less than 1.0 for afterburning engines, bpr = {}".format(self.bpr)))
            self.w = 0.063 * (self.max_thrust ** 1.1) * (self.max_mach ** 0.25) * exp(-0.81 * self.bpr)
            self.l = 0.255 * (self.max_thrust ** 0.4) * (self.max_mach ** 0.2)
            self.d = 0.024 * (self.max_thrust ** 0.5) * exp(0.04 * self.bpr)
            self.sfc_max = 2.1 * exp(-0.12 * self.bpr)
            self.t_cruise = 2.4 * (self.max_thrust ** 0.74) * exp(0.023 * self.bpr)
            self.sfc_cruise = 1.04 * exp(-0.186 * self.bpr)
        else:
            self.w = 0.084 * (self.max_thrust ** 1.1) * exp(-0.045 * self.bpr)
            self.l = 0.185 * (self.max_thrust ** 0.4) * (self.max_mach ** 0.2)
            self.d = 0.033 * (self.max_thrust ** 0.5) * exp(0.04 * self.bpr)
            self.sfc_max = 0.67 * exp(-0.12 * self.bpr)
            self.t_cruise = 0.60 * (self.max_thrust ** 0.9) * exp(0.02 * self.bpr)
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
                raise ValueError("Must specify Mach number or speed (in ft/sec)")
            mach = speed / self.atmosphere.speed_of_sound(altitude)

        density_ratio = self.atmosphere.density(altitude) / self.atmosphere.density_sl

        return a1 * (a2 + a3 * (sign * mach - a4) ** a5) * density_ratio ** a6


class Payload(object):
    def __init__(self, name=None, weight=0.0, *args, **kwargs):
        self.name = name
        self.weight = weight

    def __repr__(self):
        return "<Payload {} ({} lbm)>".format(self.name, self.weight)
