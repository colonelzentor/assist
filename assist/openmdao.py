from assist.environment import Atmosphere
from assist.aircraft import Aircraft
from assist.component import Wing, Engine, Payload
from assist.mission import Mission, Segment

from openmdao.main.api import Component
from openmdao.lib.datatypes.api import Float, Bool, Enum


class Fighter(Component):

    k_aero = Float(0.5, low=0, high=1)
    taper_ratio = Float(0.2, low=0, high=1)
    sweep = Float(30, low=0, high=55)

    tofl = Float(1500, low=0, units='ft')
    airfield_altitude = Float(0, low=0, units='ft')

    cruise_altitude = Float(30000, low=1000, units='ft')
    cruise_speed = Float(700, low=0, units='knot')
    cruise_range = Float(150, low=0, units='nmi')

    dash_altitude = Float(30000, low=1000, units='ft')
    dash_speed = Float(1492, low=0, units='knot')
    dash_range = Float(100, low=0, units='nmi')

    ldgfl = Float(1500, low=0, units='ft')
    landing_speed = Float(150, low=0, units='knot')

    def execute(self):
        wing = Wing(flap_type='single_slot',
            configuration='landing',
            slats=True,
            k_aero=self.k_aero,
            sweep=self.sweep,
            flap_span=[0.2, 0.4],
            taper_ratio=self.taper_ratio)

        self.aircraft = Aircraft(wing=wing,
                                 stores=[Payload('Crew', weight=200),
                                         Payload('Cannon', weight=270),
                                         Payload('Ammunition Feed System', weight=405),
                                         Payload('Ammunition', weight=550),
                                         Payload('Casings', weight=198),
                                         Payload('AMRAAMs', weight=332, quantity=4, cd_r=0.005, expendable=True),
                                         Payload('AIM-9Xs', weight=188, quantity=2, cd_r=0.002, expendable=True)],
                                 drag_chute=None)#{'diameter': 15.6, 'cd': 1.4})

        mission = Mission(segments=[Segment('warmup',
                                            altitude=self.airfield_altitude,
                                            speed=0,
                                            time=60),
                                    Segment('takeoff',
                                            altitude=self.airfield_altitude,
                                            speed=150,
                                            field_length=self.tofl,
                                            temperature=100),
                                    Segment('climb',
                                            altitude=self.airfield_altitude,
                                            speed=500),
                                    Segment('cruise',
                                            altitude=self.cruise_altitude,
                                            speed=self.cruise_speed,
                                            range=self.cruise_range,
                                            release=[()]),
                                    Segment('descend',
                                            altitude=self.dash_altitude,
                                            speed=1000),
                                    Segment('dash',
                                            altitude=self.dash_altitude,
                                            speed=self.dash_speed,
                                            range=self.dash_range),
                                    Segment('climb',
                                            altitude=self.cruise_altitude,
                                            speed=1000),
                                    Segment('cruise',
                                            altitude=self.cruise_altitude,
                                            speed=1050,
                                            range=self.cruise_range),
                                    Segment('descend',
                                            altitude=self.airfield_altitude,
                                            speed=1000),
                                    Segment('land',
                                            altitude=self.airfield_altitude,
                                            speed=self.landing_speed,
                                            field_length=self.ldgfl)])

        self.aircraft._synthesize(mission)
        self.aircraft._size(mission)

        self.weight = self.aircraft.w_to
        self.wing_area = self.aircraft.wing.area
        self.thrust = self.aircraft.engine.max_thrust


class AircraftSizing(Component):
    """
    based on:

    http://www.dept.aoe.vt.edu/~mason/Mason_f/ModernAircraftDesignWHM.pdf

    """

    propulsion = Enum(0, options=(1, 2), aliases=('jet', 'propeller'), desc='Propulsion system for the aircraft')
    ab = Bool(False, desc='Do(es) the engine(s) ha(s/ve) afterburner(s)?')

    w_payload = Float(low=0.0, units='lbf',
                      desc='The net carrying capacity of the aircraft, e.g., luggage, cargo, passengers, baggage, store, equipment.')
    w_crew = Float(low=0.0, units='lbf', desc='The weight of the crew')

    t_sl = Float(low=0.0, exclude_low=True, desc='Total installed maximum thrust at sea level for the aircraft', units='lbf')
    w_to = Float(low=0.0, exclude_low=True, desc='Take-off Weight', units='lbf')
    s = Float(50.0, low=0.0, exclude_low=True, desc='Wing area', units='m**2')

    alpha = Float(low=0.0, exclude_low=True, desc='Thrust Lapse')
    beta = Float(low=0.0, exclude_low=True, desc='Instantaneous Weight Fraction')

    b = Float(10.0, low=0.0, exclude_low=True, desc='Wingspan', units='m')
    e = Float(0.9, low=0.0, exclude_low=True, desc='Aerodynamic efficiency')

    c_d0 = Float(0.02, low=0.0, desc='Coefficient of drag at zero-lift')
    k1 = Float(0.0, low=0.0, desc='Inviscid drag due to lift')
    k2 = Float(0.0, low=0.0, desc='Viscous drag due to lift')

    v_cruise = Float(desc='Velocity at cruise', units='knot')
    endurance = Float(desc='Time aircraft can remain aloft', units='hr')

    def execute(self):
        self.fuel_w_frac = 1.05 * (1 - 0) # TODO: fix this!
        self.w_to = (self.w_payload + self.w_crew) / (1 - self.fuel_w_frac - self.empty_w_frac)

        self.ar = self.b * self.b / self.s_ref
        self.c_d = self.c_d0 + self.c_l * self.c_l / (pi * self.ar * self.e)

        self.v_cruise = sqrt(2.0 * self.t_sl * self.alpha / ((self.c_d0 + self.c_dr) * self.s))

        if self.propulsion == 1:
            self.endurance = (1 / c_t) * (L / D) * log()
        elif self.propulsion == 2:
            self.endurance =


class JetEngineSizing(Component):
    """
    Determines a jet engine's parameters based on Raymer's rules as defined in:

    Raymer, D. P., "Aircraft design: a conceptual approach", 3rd Ed., pp. 235

    .. note::
        Cruise is assumed to be at approximately 36,000 ft (11,000 m) and 0.9 Mach

    """

    afternburner = Bool(False, iotype='in', desc="Does the engine have afterburners?")
    m = Float(1.0, iotype='in', low=0.0, exclude_low=True, high=2.5, desc="Maximum Mach number")
    bpr = Float(1.0, iotype='in', low=0.0, high=6.0, exclude_low=True, desc="Bypass Ratio")
    t = Float(1.0, iotype='in', low=0.0, exclude_low=True, desc="Takeoff Thrust", units='lbf')

    # k-factors
    k_sfc = Float(1.0, iotype='in', low=0.0, exclude_low=True, high=1.0, desc="k-factor for engine SFC")
    k_w = Float(1.0, iotype='in', low=0.0, high=1.0, desc="k-factor for engine weight")
    k_size = Float(1.0, iotype='in', low=0.0, high=1.0, desc="k-factor for engine size")

    w = Float(1.0, iotype='out', low=0.0, exclude_low=True, desc="Scaled Engine Weight", units='lbm')
    l = Float(1.0, iotype='out', low=0.0, exclude_low=True, desc="Scaled Engine Length", units='ft')
    d = Float(1.0, iotype='out', low=0.0, exclude_low=True, desc="Scaled Engine Diameter", units='ft')
    sfc_max = Float(1.0, iotype='out', low=0.0, exclude_low=True, desc="Scaled Specific Fuel Consumption at Maximum Thrust", units='1/hr')
    t_cruise = Float(1.0, iotype='out', low=0.0, exclude_low=True, desc="Scaled Thrust at Cruise", units='lbf')
    sfc_cruise = Float(1.0, iotype='out', low=0.0, exclude_low=True, desc="Scaled Specific Fuel Consumption at Cruise", units='1/hr')

    def execute(self):
        if self.afternburner:
            if self.bpr > 1.0:
                raise(NotImplementedError("BPR must be less than 1.0 for afterburning engines, bpr = {}".format(self.bpr)))
            self.w = 0.063 * (self.t ** 1.1) * (self.m ** 0.25) * exp(-0.81 * self.bpr)
            self.l = 0.255 * (self.t ** 0.4) * (self.m ** 0.2)
            self.d = 0.024 * (self.t ** 0.5) * exp(0.04 * self.bpr)
            self.sfc_max = 2.1 * exp(-0.12 * self.bpr)
            self.t_cruise = 2.4 * (self.t ** 0.74) * exp(0.023 * self.bpr)
            self.sfc_cruise = 1.04 * exp(-0.186 * self.bpr)
        else:
            self.w = 0.084 * (self.t ** 1.1) * exp(-0.045 * self.bpr)
            self.l = 0.185 * (self.t ** 0.4) * (self.m ** 0.2)
            self.d = 0.033 * (self.t ** 0.5) * exp(0.04 * self.bpr)
            self.sfc_max = 0.67 * exp(-0.12 * self.bpr)
            self.t_cruise = 0.60 * (self.t ** 0.9) * exp(0.02 * self.bpr)
            self.sfc_cruise = 0.88 * exp(-0.05 * self.bpr)

        self.w = self.k_w * self.w
        self.sfc_max = self.k_sfc * self.sfc_max
        self.sfc_cruise = self.k_sfc * self.sfc_cruise
        self.l = self.k_size * self.l
        self.d = self.k_size * self.d


class AircraftCost(Component):
    """
    This is based on the Modified DAPCA IV Cost model presented in Raymer, 1999

    Cost in constant 1999 U$D.

    TODO: add life-cycle cost calculations (Raymer's is focused on commercial aircraft)

    """
    cargo = Bool(False, iotype='in', desc="Is this a cargo aircraft?")
    spares = Bool(False, iotype='in', desc="Acquire initial spares?")
    profit = Float(1.2, iotype='in', low=1, high=1.5, desc="Profit/Investment Cost Factor")
    quantity = Int(100, iotype='in', desc="Lesser of production quantity or number to be produced in 5 years")
    num_flight_test_aircraft = Int(2, iotype='in', desc="Number of flight test aircraft")

    empty_weight = Float(iotype='in', desc="empty weight", units='lbm')
    max_velocity = Float(iotype='in', desc="Maximum velocity", units='kts')
    max_thrust = Float(iotype='in', desc="Engine maximum thrust", units='lbf')
    max_mach = Float(iotype='in', desc="Engine maximum Mach number")
    stealth = Float(0.0, iotypes='in', low=0, high=1, desc="Degree of stealth complexity, 0 indicates no low-observable components/materials")
    materials_complexity = Float(1.0, iotypes='in', low=1, high=2, desc="Materials degree of complexity, aluminum: 1.0, graphite-epoxy: 1.1-1.8, fiberglass: 1.1-1.2, steel: 1.5-2.0, titanium: 1.3-2.0"))

    avionics_weight = Float(iotype='in', desc="Cost of avionics", units='lbm')
    avionics_complexity = Float(0.25, iotype='in', low=0, high=1, desc="Complexity of the avionics")

    num_engines = Int(iotype='in', desc="Number of engines")
    turbine_inlet_temp = Float(iotype='in', desc="Turbine inlet temperature", units='degR')

    r_eng = Float(86, iotype='in', desc="Cost per hour of engineering work", units='USD')
    r_tng = Float(88, iotype='in', desc="Cost per hour of tooling work", units='USD')
    r_mfg = Float(73, iotype='in', desc="Cost per hour of manufacturing work", units='USD')
    r_qyc = Float(81, iotype='in', desc="Cost per hour of quality control work", units='USD')

    year = Int(2014, low=1990, iotype='in', desc="Year for cost (original model is in 1999 USD)", units='y')

    acq_cost = Float(iotype='out', desc="Acquisition cost, i.e., total RDT&E plus flyaway cost", units='USD')

    def execute(self):
        w_e = self.empty_weight
        v = self.max_velocity
        q = self.max_quantity

        if self.year in ESCALATION:
            f_mfg = ESCALATION[self.year][0]
            f_eng = ESCALATION[self.year][1]
            f_oth = ESCALATION[self.year][2]
        else:
            f_mfg = FUTURE_ESCALATIONS['aircraft'](self.year)
            f_eng = FUTURE_ESCALATIONS['engine'](self.year)
            f_oth = FUTURE_ESCALATIONS['other'](self.year)

        h_mult = self.materials_complexity
        if self.stealth > 0.0:
            h_mult *= 1.20 + 0.2 * self.stealth

        h_e = h_mult * 7.070 * w_e ** 0.777 * v ** 0.894 * q ** 0.163
        h_t = h_mult * 8.710 * w_e ** 0.777 * v ** 0.696 * q ** 0.263
        h_m = h_mult * 10.72 * w_e ** 0.820 * v ** 0.484 * q ** 0.641
        if self.cargo:
            h_qc = 0.076 * h_m
        else:
            h_qc = 0.133 * h_m

        c_d = 66.0 * w_e ** 0.630 * v ** 1.3
        c_f = 1807.1 * w_e ** 0.325 * v ** 0.822 * self.num_flight_test_aircraft ** 1.21
        c_m = 16 * w_e ** 0.921 * v ** 0.621 * q ** 0.799
        c_eng = 2215 * (0.043 * self.max_thrust + 243.25 * self.max_mach + 0.969 * self.turbine_inlet_temp - 2228)

        c_avionics = self.avionics_weight * (3000 + 3000 * self.avionics_complexity)

        self.cost = h_e * self.r_eng * f_mfg + \
                    h_t * self.r_tng * f_mfg  + \
                    h_m * self.r_mfg * f_mfg  + \
                    h_qc * self.r_qyc * f_mfg  + \
                    c_d * f_mfg + \
                    c_f * f_oth + \
                    c_m * f_mfg + \
                    c_eng * self.num_engines * f_eng + \
                    c_avionics * f_oth

        self.cost *= self.profit

        if self.spares:
            self.cost *= 1.125


class AircraftCost(Component):
    cargo = Bool(False, iotype='in', desc="Is this a cargo aircraft?")
    spares = Bool(False, iotype='in', desc="Acquire initial spares?")
    profit = Float(1.2, iotype='in', low=1, high=1.5, desc="Profit/Investment Cost Factor")
    quantity = Int(100, iotype='in', desc="Lesser of production quantity or number to be produced in 5 years")
    num_flight_test_aircraft = Int(2, iotype='in', desc="Number of flight test aircraft")

    empty_weight = Float(iotype='in', desc="empty weight", units='lbm')
    max_velocity = Float(iotype='in', desc="Maximum velocity", units='kts')
    max_thrust = Float(iotype='in', desc="Engine maximum thrust", units='lbf')
    max_mach = Float(iotype='in', desc="Engine maximum Mach number")
    stealth = Float(0.0, iotypes='in', low=0, high=1, desc="Degree of stealth complexity, 0 indicates no low-observable components/materials")
    materials_complexity = Float(1.0, iotypes='in', low=1, high=2, desc="Materials degree of complexity, aluminum: 1.0, graphite-epoxy: 1.1-1.8, fiberglass: 1.1-1.2, steel: 1.5-2.0, titanium: 1.3-2.0"))

    avionics_weight = Float(iotype='in', desc="Cost of avionics", units='lbm')
    avionics_complexity = Float(0.25, iotype='in', low=0, high=1, desc="Complexity of the avionics")

    num_engines = Int(iotype='in', desc="Number of engines")
    turbine_inlet_temp = Float(iotype='in', desc="Turbine inlet temperature", units='degR')

    r_eng = Float(86, iotype='in', desc="Cost per hour of engineering work", units='USD')
    r_tng = Float(88, iotype='in', desc="Cost per hour of tooling work", units='USD')
    r_mfg = Float(73, iotype='in', desc="Cost per hour of manufacturing work", units='USD')
    r_qyc = Float(81, iotype='in', desc="Cost per hour of quality control work", units='USD')

    year = Int(2014, low=1990, iotype='in', desc="Year for cost (original model is in 1999 USD)", units='y')

    acq_cost = Float(iotype='out', desc="Acquisition cost, i.e., total RDT&E plus flyaway cost", units='USD')
