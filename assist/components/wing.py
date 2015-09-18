from __future__ import division
from numpy import sqrt, exp
from assist.util import verify_value
from assist.environment import Atmosphere


__all__ = ('Wing',)


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
