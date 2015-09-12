from __future__ import division, print_function
from warnings import warn

from numpy import array, pi, exp, sqrt, log, max, argmin, cos, sin, abs
from scipy.interpolate import interp1d

from environment import Atmosphere
from component import Wing, Engine


class Aircraft(object):
    """
    Conceptual-level Aircraft definition.

    :param sweep: sweep of the wing at the quarter-chord
    :type sweep: list, tuple, float

    """

    # TODO: get a better way to estimate the T/W initially
    _T_TO_W = {
        'jet_trainer': 0.75,
        'jet_fighter': 1.20,
        'mil_cargo': 0.45,
        'bomber': 0.65,
        'jet_transport': 0.35
    }

    # Empty weight coefficients, from Raymer, 1999, pp. 115
    _W_E_TO_W_TO_COEFFICIENTS = {
        'jet_trainer': (0.00, 4.28, -0.10, 0.10, 0.20, -0.24, 0.11),
        'jet_fighter': (-0.02, 2.16, -0.10, 0.20, 0.04, -0.10, 0.08),
        'mil_cargo': (0.07, 1.71, -0.10, 0.10, 0.06, -0.10, 0.05),
        'bomber': (0.07, 1.71, -0.10, 0.10, 0.06, -0.10, 0.05),
        'jet_transport': (0.32, 0.66, -0.13, 0.30, 0.06, -0.05, 0.05)
    }

    _DESIGN_MACH = {
        'jet_trainer': 0.95,
        'jet_fighter': 1.50,
        'mil_cargo': 0.85,
        'bomber': 0.92,
        'jet_transport': 0.78
    }

    _CD_0 = {
        'jet_fighter': {
            'mach': [0.0, 0.8, 0.9, 1.1, 1.2, 2.0],
            'min': [0.014, 0.014, 0.0160, 0.0260, 0.028, 0.028],
            'max': [0.018, 0.018, 0.0235, 0.0345, 0.040, 0.038]
        }
    }

    _K_1 = {
        'jet_fighter': {
            'mach': [0.0, 0.8, 1.0, 1.2, 2.0],
            'min': [0.180, 0.180, 0.180, 0.216, 0.360],
            'max': [0.140, 0.140, 0.170, 0.200, 0.500]
        }
    }

    _CONFIGURATIONS = ('takeoff', 'landing', 'cruise')

    def __init__(self,
                 aircraft_type='jet_fighter',
                 wing=None,
                 engine='ATJ',
                 num_engines=1,
                 stores=None,
                 k_aero=0.5,
                 k_2=0.003,
                 cd_r=None,
                 k_to=1.1,
                 reverse_thrust=False,
                 drag_chute=None,
                 k_td=1.15, *args, **kwargs):

        self.type = aircraft_type
        self.design_mach = self._DESIGN_MACH[aircraft_type]
        self.wing = Wing(k_aero=k_aero,
                         aircraft_type=aircraft_type,
                         design_mach=self.design_mach) if not isinstance(
                             wing, Wing) else wing
        self.configuration = self.wing.configuration
        self.engine = Engine(engine) if isinstance(
            engine, basestring) else engine
        self.num_engines = num_engines
        self._cd_r = {'takeoff': 0.02, 'landing': 0.02, 'cruise': 0.0}

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

        self.reverse_thrust = reverse_thrust

        self.drag_chute = None
        if drag_chute is not None:
            self.drag_chute = dict(diameter=15.0, cd=1.4)
            self.drag_chute.update(drag_chute)

        if self._cd_0 is None and aircraft_type in self._CD_0:
            self._cd_0_min = interp1d(self._CD_0[aircraft_type]['mach'],
                                      self._CD_0[aircraft_type]['min'],
                                      bounds_error=False,
                                      fill_value=0.02)
            self._cd_0_max = interp1d(self._CD_0[aircraft_type]['mach'],
                                      self._CD_0[aircraft_type]['max'],
                                      bounds_error=False,
                                      fill_value=0.02)
        else:
            self._cd_0 = 0.02

        if self._k_1 is None and aircraft_type in self._K_1:
            self._k_1_min = interp1d(self._K_1[aircraft_type]['mach'],
                                     self._K_1[aircraft_type]['min'],
                                     bounds_error=False,
                                     fill_value=0.02)
            self._k_1_max = interp1d(self._K_1[aircraft_type]['mach'],
                                     self._K_1[aircraft_type]['max'],
                                     bounds_error=False,
                                     fill_value=0.02)
        else:
            self._k_1 = 0.16

        self.k_to = k_to
        self.k_td = k_td

        self.cl_max = self.wing.cl_max
        self.takeoff = self.wing.takeoff
        self.landing = self.wing.landing
        self.cruise = self.wing.cruise
        self.thrust_lapse = self.engine.thrust_lapse

    def __repr__(self):
        return "<Aircraft {} ({}, {})>".format(self.type, str(self.wing), str(
            self.engine))

    @property
    def payload(self):
        total = 0
        for store in self.stores:
            total += store.weight
        return total

    @property
    def cd_r(self):
        cd_r = 0.0
        if self.configuration in self._cd_r:
            cd_r += self._cd_r[self.configuration]
        return cd_r + sum(store.cd_r for store in self.stores)

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

    @property
    def cd(self):
        cl = getattr(self, 'cl', 0.0)
        return self.k_1 * cl * cl + self.k_2 * cl + self.cd_0

    def synthesize(self, mission, wing_loading=None):
        """
        Identifies a design point for a mission

        """

        wing_loadings = array(range(10, 300))
        thrust_loadings = []
        weight_fraction = 1.0

        self.max_mach = 0
        for segment in mission.segments:
            if segment.mach > self.max_mach:
                self.max_mach = segment.mach
                self.mach = segment.mach
            thrust_loadings.append(segment.thrust_to_weight_required(
                aircraft=self,
                wing_loading=wing_loadings,
                prior_weight_fraction=weight_fraction))
            weight_fraction *= segment.weight_fraction
            print("Segment {} has a weight fraction of {}".format(
                segment.kind, segment.weight_fraction))

        self.fuel_fraction = 1 - weight_fraction

        self._synthesis = {'w_to_s': wing_loadings,
                           't_to_w': thrust_loadings}

        self.t_to_w_req = max(array(zip(*thrust_loadings)), 1)
        idx = argmin(self.t_to_w_req)

        self.t_to_w = self.t_to_w_req[idx]
        self.w_to_s = wing_loadings[idx]

    def size(self, mission, w_to=(1000, 60000), tol=10):
        """
        Sizes the aircraft for a given mission

        Empty Weight Fraction (we_to_w0) based on (Raymer, 1999) pp. 115

        """

        if hasattr(w_to, '__iter__'):
            w_to = array(range(w_to[0], w_to[1], tol))

        if self.type not in coefficients:
            raise NotImplementedError(
                "Aircraft type '{}' not implemented, " +
                "only these have been implemented: {}".format(
                    self.type, coefficients.keys()))

        for segment in mission.segments:
            pass
        wf_to_w0 = 1 - mission.segments[-1].prior_weight_fraction * \
            mission.segments[-1].weight_fraction

        a, b, c1, c2, c3, c4, c5 = self._W_E_TO_W_TO_COEFFICIENTS[self.type]
        k_vs = 1.04 if self.variable_sweep else 1.0

        we_to_w0 = (a + b * w_to ** c1 * self.wing.aspect_ratio ** c2 *
                    self.t_to_w ** c3 * self.w_to_s ** c4 * self.max_mach ** c5
                    ) * k_vs

        w_to_calc = self.payload / (wf_to_w0 - we_to_w0)

        idx = argmin(abs(w_to_calc - w_to))

        self.w_to = w_to_calc[idx]

        self.engine.max_mach = self.max_mach
        self.engine.max_thrust = self.t_to_w * self.w_to / self.num_engines
        self.wing.area = self.w_to / self.w_to_s
