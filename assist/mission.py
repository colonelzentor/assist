from __future__ import division

from math import sqrt, exp

from environment import Atmosphere, G_0


class Segment(object):
    """
    Aircraft mission

    :param name: the name of the type of segment
    :param speed: the speed at which the segment is to be flown (knots)
    :param altitude: the altitude at which the segment will take place (ft)
    :param atmosphere: the atmosphere instance that contains the sea level conditions, if None s provided, a standard one is created

    :type name: str
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

    _WEIGHT_FRACTIONS = dict(warmup=0.99,
                             taxi=0.99,
                             takeoff=0.98,
                             climb=0.95,
                             descend=0.98,
                             land=0.99)

    def __init__(self, name, speed, altitude,
                 atmosphere=None,
                 *args, **kwargs):

        self.name = name
        if 'weight_fraction' not in kwargs and name in self._WEIGHT_FRACTIONS:
            self._weight_fraction = self._WEIGHT_FRACTIONS[name]
        else:
            self._weight_fraction = kwargs.pop('weight_fraction', None)

        self.altitude = altitude

        self.atmosphere = Atmosphere() if atmosphere is None else atmosphere

        self.density = self.atmosphere.density(altitude)

        if speed is not None:
            self.speed = speed * 1.68780986  # kts to ft/s
            self.mach = self.speed / self.atmosphere.speed_of_sound(self.altitude)

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
        self.afterburner = True if 'dash' in self.name else False

        self.dynamic_pressure = 0.5 * self.density * self.speed * self.speed

        if 'land' in self.name:
            self.ldgfl = kwargs.pop('field_length', 2500)
            self.approach_speed = sqrt(self.ldgfl / 0.3) * 6076.1154856 / 3600

        elif 'takeoff' in self.name:
            self.tofl = kwargs.pop('tofl', 1500)
            self.obstacle_height = kwargs.pop('obstacle_height', 100)

        elif 'cruise' in self.name or 'dash' in self.name:
            self.time = kwargs['range'] / speed

        elif 'loiter' in self.name:
            self.time = kwargs['loiter_time']

    @property
    def weight_fraction(self):
        if self._weight_fraction is not None:
            return self._weight_fraction
        else:
            tsfc = self.aircraft.engine.tsfc(self.mach, self.altitude, self.afterburner)
            t_to_w = self.aircraft.t_to_w * self.aircraft.thrust_lapse(self.altitude, self.mach) / self.prior_weight_fraction
            return 1 - exp(-tsfc * t_to_w * self.time)

    def thrust_to_weight_required(self, aircraft, wing_loading, prior_weight_fraction=1):
        self.aircraft = aircraft
        self.prior_weight_fraction = prior_weight_fraction

        cd_0 = aircraft.cd_0
        k_1 = aircraft.k_1
        k_2 = aircraft.k_2
        # TODO: calculate C_DR as a function of mission segment, i.e., stores
        cd_r = 0.02 # aircraft.cd_r

        alpha = aircraft.thrust_lapse(self.altitude, self.mach)
        beta = self.weight_fraction

        if 'land' in self.name:
            aircraft.wing.landing
            #TODO finish landing constraint

        if 'takeoff' in self.name:
            aircraft.wing.takeoff
            k_to = aircraft.k_to
            return (beta * beta / alpha) * k_to * k_to * wing_loading / \
                   (self.tofl * self.density * G_0 * aircraft.cl_max)

        aircraft.configuration = None

        q = self.dynamic_pressure
        c_l = self.n * beta * wing_loading / q
        excess_power = self.climb_rate / self.speed + self.acceleration / G_0

        # Master Equation from Mattingly, 2002
        return (beta / alpha) * (q / (beta * wing_loading) * (k_1 * c_l * c_l + k_2 * c_l + cd_0 + cd_r) + excess_power)
