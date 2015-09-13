from __future__ import division


__all__ = ('Payload',)


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
