from util import verify_value


class Cost(object):
    """
    This is based on the Modified DAPCA IV Cost model presented in Raymer, 1999

    Cost in constant 1999 U$D.

    TODO: add life-cycle cost calculations (Raymer's is focused on commercial aircraft)

    """

    _DEFAULTS = dict(cargo=[None, None, False, 'n/a'],
                     spares=[None, None, False, 'n/a'],
                     profit=[1.0, 2.0, 1.2, 'unitless'],
                     quantity=[1, None, 100, 'unitless'],
                     num_flight_test_aircraft=[1, None, 2, 'n/a'],
                     empty_weight=[0, None, None, 'lbm'],
                     max_velocity=[1, None, None, 'kts'],
                     max_thrust=[1, None, None, 'lbf'],
                     max_mach=[0.0, None, None, 'unitless'],
                     stealth=[0.0, 1.0, 0.1, 'unitless'],
                     materials_complexity=[1.0, 2.0, 1.0, 'unitless'],
                     avionics_weight=[0.0, None, 0.0, 'unitless'],
                     avionics_complexity=[0.0, 1.0, 0.25, 'unitless']
                     num_engines=[1, None, 1, 'unitless'],
                     turbine_inlet_temp=[0.0, None, None, 'degR'],
                     r_eng = [0, None, 86, 
                     r_tng = 88,
                     r_mfg = 73,
                     r_qyc = 81,
                     year = 2015,)

    # Based on http://www.aia-aerospace.org/research_reports/aerospace_statistics/
    #                YEAR  Aircraft            Engine/Eng Parts   Other Parts/Equip
    _ESCALATIONS = { 1990: [0.918708240534521, 0.822946175637394, 0.818615751789976],
                     1991: [0.948775055679287, 0.861189801699717, 0.846062052505967],
                     1992: [0.968819599109131, 0.903682719546742, 0.878281622911694],
                     1993: [0.983296213808463, 0.919263456090652, 0.900954653937947],
                     1994: [0.996659242761693, 0.943342776203966, 0.920047732696897],
                     1995: [1.005567928730510, 0.956090651558074, 0.927207637231504],
                     1996: [1.010022271714920, 0.974504249291785, 0.959427207637232],
                     1997: [1.003340757238310, 0.985835694050991, 0.978520286396181],
                     1998: [0.998886414253898, 0.992917847025496, 0.990453460620525],
                     1999: [1, 1, 1],
                     2000: [1.008908685968820, 1.021246458923510, 1.008353221957040],
                     2001: [1.015590200445430, 1.052407932011330, 1.031026252983290],
                     2002: [1.016703786191540, 1.065155807365440, 1.041766109785200],
                     2003: [1.035634743875280, 1.117563739376770, 1.038186157517900],
                     2004: [1.056792873051230, 1.172804532577900, 1.041766109785200],
                     2005: [1.074610244988860, 1.195467422096320, 1.068019093078760],
                     2006: [1.106904231625840, 1.240793201133140, 1.082338902147970],
                     2007: [1.102449888641430, 1.294617563739380, 1.124105011933170],
                     2008: [1.106904231625840, 1.345609065155810, 1.169451073985680],
                     2009: [1.113585746102450, 1.416430594900850, 1.193317422434370],
                     2010: [1.112472160356350, 1.454674220963170, 1.190930787589500],
                     2011: [1.116926503340760, 1.495750708215300, 1.202863961813840],
                     2012: [1.125835189309580, 1.536827195467420, 1.207637231503580],
                     2013: [1.135857461024500, 1.563739376770540, 1.232696897374700],
                     2014: [1.140311804008910, 1.580736543909350, 1.248210023866350]}

    _FUTURE_ESCALATIONS = dict(aircraft=lambda yr: 0.0169864145 * yr - 32.9627923628,
                               engines=lambda yr: 0.0386483205 * yr - 76.2369486042,
                               other=lambda yr: 0.0084778139 * yr - 15.9284409799)

    def __init__(self, *args, **kwargs):

    # RDT&E + Fly Away cost
    def estimate_acquisition(self):
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

        self.acquisition_cost = h_e * self.r_eng * f_mfg + \
                                h_t * self.r_tng * f_mfg  + \
                                h_m * self.r_mfg * f_mfg  + \
                                h_qc * self.r_qyc * f_mfg  + \
                                c_d * f_mfg + \
                                c_f * f_oth + \
                                c_m * f_mfg + \
                                c_eng * self.num_engines * f_eng + \
                                c_avionics * f_oth

        self.acquisition_cost *= self.profit

        if self.spares:
            self.acquisition_cost *= 1.125

        return self.acquisition_cost
