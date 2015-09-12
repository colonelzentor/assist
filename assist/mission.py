from __future__ import division
from warnings import warn
from numpy import sqrt, exp, power, linspace, interp, log, pi
from environment import Atmosphere, G_0


MAX_T_TO_W = 5


class Mission(object):
    """
    A mission as defined by a list of segments.

    """

    def __init__(self, segments=None, atmosphere=None, *args, **kwargs):
        self.atmosphere = Atmosphere() if atmosphere is None else atmosphere

        if segments is not None:
            self.segments = segments
        else:
            raise NotImplementedError("A mission generator has not been implemented yet, must provide list of segments.")


class Segment(object):
    """
    Aircraft mission

    :param kind: the type of segment, e.g., takeoff, cruise, dash, loiter, land
    :param speed: the speed at which the segment is to be flown (knots)
    :param altitude: the altitude at which the segment will take place (ft)
    :param atmosphere: the atmosphere instance that contains the sea level conditions, if None s provided, a standard one is created

    :type kind: str
    :type speed: float
    :type altitude: float
    :type atmosphere: ::class::`Atmosphere`

    If mission is of type `cruise`:
    :param range: the range to fly during the segment (nmi)
    :type range: float

    If mission is of type `loiter`:
    :param loiter_time: time to loiter (hrs)
    :type loiter_time: float

    """

    _DEFAULTS = dict(warmup=dict(time=60.0),
                     takeoff=dict(field_length= 1500,
                                  mu=0.05,
                                  time=3,
                                  obstacle_height=100),
                     land=dict(field_length=2500,
                               mu=0.18,
                               time=3,
                               obstacle_height=100),
                     loiter=dict(time=None),
                    )

    _WEIGHT_FRACTIONS = dict(warmup=0.99,
                             taxi=0.99,
                             takeoff=0.98,
                             climb=0.95,
                             descend=0.98,
                             land=0.99,
                            )

    def __init__(self, kind, speed, altitude, payload_released=0,
                 atmosphere=None,
                 release=None, *args, **kwargs):

        self.kind = kind
        if 'weight_fraction' not in kwargs and kind in self._WEIGHT_FRACTIONS:
            self._weight_fraction = self._WEIGHT_FRACTIONS[kind]
        else:
            self._weight_fraction = kwargs.pop('weight_fraction', None)

        self.altitude = altitude

        self.payload_released = payload_released

        self.atmosphere = Atmosphere() if atmosphere is None else atmosphere

        self.density = self.atmosphere.density(altitude)

        self.release = release

        if speed is not None:
            self.speed = speed * 1.68780986  # kts to ft/s
            self.mach = self.speed / \
                self.atmosphere.speed_of_sound(self.altitude)

        self.n = 1
        if 'turn_rate' in kwargs:
            turn_rate = kwargs.pop('turn_rate')
            self.n = sqrt(1 + (turn_rate * self.speed / G_0) ** 2)
        if 'turn_radius' in kwargs:
            turn_radius = kwargs.pop('turn_radius')
            n = sqrt(1 + (self.speed / turn_radius / G_0) ** 2)
            if hasattr(self, 'n'):
                self.n = max(n, self.n)

        self.climb_rate = kwargs.pop('climb_rate', 0)
        self.acceleration = kwargs.pop('acceleration', 0)

        self.dynamic_pressure = 0.5 * self.density * self.speed * self.speed

        for key, defaults in self._DEFAULTS.items():
            if key in self.kind:
                for var, default in defaults.items():
                    setattr(self, var, kwargs.pop(var, default))

        if 'cruise' in self.kind or 'dash' in self.kind:
            self.range = kwargs.pop('range')
            self.time = self.range / speed

        if len(kwargs) > 0:
            warn("Unused kwargs: {}".format(kwargs.keys()))

    @property
    def weight_fraction(self):
        if self._weight_fraction is not None:
            return self._weight_fraction
        else:
            tsfc = self.aircraft.engine.tsfc(
                self.mach, self.altitude, self.afterburner)
            t_to_w = self.aircraft.t_to_w * \
                self.aircraft.thrust_lapse(
                    self.altitude, self.mach) / self.prior_weight_fraction
            return 1 - exp(-tsfc * t_to_w * self.time)

    def thrust_to_weight_required(self, aircraft, wing_loading, prior_weight_fraction=1):
        if self.speed == 0:
            return [0.0] * len(wing_loading) if hasattr(wing_loading, '__iter__') else 0.0
        self.aircraft = aircraft
        self.prior_weight_fraction = prior_weight_fraction
        self.afterburner = self.aircraft.engine.afterburner and 'dash' in self.kind

        cd_0 = aircraft.cd_0
        k_1 = aircraft.k_1
        k_2 = aircraft.k_2

        if self.release is not None:
            self.aircraft.stores = [store for store in self.aircraft.stores if store not in self.release]

        alpha = aircraft.thrust_lapse(self.altitude, self.mach)
        beta = self.prior_weight_fraction

        t_to_w = None
        if 'takeoff' in self.kind:
            aircraft.takeoff
            k_to = aircraft.k_to
            cl_max = self.aircraft.cl_max
            self.aircraft.cl = cl = cl_max / (k_to * k_to)
            xi = self.aircraft.cd + aircraft.cd_r - self.mu * self.aircraft.cl

            t_to_w = linspace(0.01, MAX_T_TO_W, 200)

            a = k_to * k_to * beta * beta / (self.density * G_0 * cl_max * alpha * t_to_w)
            a = - (beta / (self.density * G_0 * xi)) * log(1 - xi / ((alpha * t_to_w / beta - self.mu) * cl))
            b = self.time * k_to * sqrt(2 * beta / (self.density * cl_max))
            c = self.field_length

            w_to_s = power((-b + sqrt(b * b + 4 * a * c)) / (2 * a), 2)

            self.aircraft._takeoff  = {'w_to_s': w_to_s, 't_to_w': t_to_w, 'a': a, 'b': b, 'c': c}

            return interp(wing_loading, w_to_s, t_to_w)

        if 'land' in self.kind:
            aircraft.landing
            k_td = self.aircraft.k_td
            cl_max = self.aircraft.cl_max
            self.aircraft.cl = cl = cl_max / (k_td * k_td)

            if aircraft.reverse_thrust:
                alpha = -alpha
            else:
                alpha = 0.0

            # assume drag chute
            cd_chute = 0.0
            if self.aircraft.drag_chute is not None:
                drag_chute_diam = self.aircraft.drag_chute['diameter']
                drag_chute_cd = self.aircraft.drag_chute['cd']
                try:
                    wing_area = self.aircraft.wing.area
                except AttributeError:
                    wing_area = 500
                    warn("Could not get an area for the wing (self.aircraft.wing.area), assuming 500 sqft")
                cd_chute = drag_chute_cd * 0.25 * drag_chute_diam * drag_chute_diam * pi / wing_area

            xi = self.aircraft.cd + aircraft.cd_r - self.mu * self.aircraft.cl + cd_chute

            t_to_w = linspace(0.01, MAX_T_TO_W, 200)

            a = (beta / (self.density * G_0 * xi)) * log(1 + xi / ((self.mu + (alpha / beta) * t_to_w) * cl))
            b = self.time * k_td * sqrt(2 * beta / (self.density * cl_max))
            c = self.field_length

            w_to_s = power((-b + sqrt(b * b + 4 * a * c)) / (2 * a), 2)

            self.aircraft._land = {'w_to_s': w_to_s, 't_to_w': t_to_w, 'a': a, 'b': b, 'c': c}

            return interp(wing_loading, w_to_s, t_to_w)

        cd_r = aircraft.cd_r

        aircraft.configuration = None

        q = self.dynamic_pressure
        c_l = self.n * beta * wing_loading / q
        excess_power = self.climb_rate / self.speed + self.acceleration / G_0

        # Master Equation from Mattingly, 2002
        return (beta / alpha) * (q / (beta * wing_loading) * (k_1 * c_l * c_l + k_2 * c_l + cd_0 + cd_r) + excess_power)
