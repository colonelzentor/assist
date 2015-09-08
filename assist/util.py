def verify_value(name, value, min_value=None, max_value=None, units='unitless'):
    if (min_value is not None and value < min_value) or (max_value is not None and value > max_value):
        raise ValueError("Value for '{}' [{} ({})] outside of bounds [{}, {}]".format(name, value, units, min_value, max_value))
