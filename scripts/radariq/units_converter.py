# This file is part of RadarIQ SDK
# (C) 2019 RadarIQ <support@radariq.io>
#
# SPDX-License-Identifier:    MIT

"""
Conversion to/from SI units of meters and meters per second into other units.
"""
from __future__ import division
import numbers
from math import log10, floor

# Conversion factors for distance conversion (from meters)
distance_lookup = {
    "mm": 1000,
    "cm": 100,
    "m": 1,
    "km": (1 / 1000),
    "in": 39.3701,
    "ft": 3.28084,
    "mi": (1 / 1609.344)
}

# Conversion factors for speed conversion (from meters/second)
speed_lookup = {
    "mm/s": 1000,
    "cm/s": 100,
    "m/s": 1,
    "km/h": 3.6,
    "in/s": 39.3701,
    "ft/s": 3.28084,
    "mi/h": 2.237
}

# Conversion factors for acceleration conversion (from meters/second)
acceleration_lookup = {
    "mm/s^2": 1000,
    "cm/s^2": 100,
    "m/s^2": 1,
    "in/s^2": 39.3701,
    "ft/s^2": 3.28084,
}


def convert_distance_to_si(units, distance):
    """
    Converts distance from another unit to meters.

    :param units: One of the units listed in the distance lookup dictionary
    :type units: str
    :param distance: The distance to convert from
    :type distance: number
    :return: The converted distance
    :rtype: float
    """

    global distance_lookup
    assert isinstance(distance, numbers.Real), "Distance must be a number"
    assert units in distance_lookup, "Invalid units for distance conversion"
    factor = distance_lookup[units]
    return round_sig(distance / factor)


def convert_distance_from_si(units, distance):
    """
    Converts distance from meters to other units.

    :param units: One of the units listed in the distance lookup dictionary
    :type units: str
    :param distance: The distance to convert from
    :type distance: number
    :return: The converted distance
    :rtype: float
    """

    global distance_lookup
    if not isinstance(distance, numbers.Real):
        raise ValueError("Distance must be a number")

    if units not in distance_lookup:
        raise ValueError("Invalid units for distance conversion")

    factor = distance_lookup[units]
    return round_sig(distance * factor)


def convert_speed_to_si(units, speed):
    """
    Converts speed from meters per second to other units.

    :param units: One of the units listed in the speed lookup dictionary
    :type units: str
    :param speed: The speed to convert from
    :type speed: number
    :return: The converted speed
    :rtype: float
    """
    global speed_lookup

    if not isinstance(speed, numbers.Real):
        raise ValueError("Speed must be a number")

    if units not in speed_lookup:
        raise ValueError("Invalid units for speed conversion")

    factor = speed_lookup[units]
    return round_sig(speed / factor)


def convert_speed_from_si(units, speed):
    """
    Converts speed from other units to metres per second.

    :param units: One of the units listed in the speed lookup dictionary
    :type units: str
    :param speed: The speed to convert from
    :type speed: number
    :return: The converted speed
    :rtype: float
    """
    global speed_lookup

    if not isinstance(speed, numbers.Real):
        raise ValueError("Speed must be a number")

    if units not in speed_lookup:
        raise ValueError("Invalid units for speed conversion")

    factor = speed_lookup[units]
    return round_sig(speed * factor)


def convert_acceleration_to_si(units, acceleration):
    """
    Converts speed from meters per square second to other units.

    :param units: One of the units listed in the speed lookup dictionary
    :type units: str
    :param acceleration: The acceleration to convert from
    :type acceleration: number
    :return: The converted acceleration
    :rtype: float
    """
    global acceleration_lookup

    if not isinstance(acceleration, numbers.Real):
        raise ValueError("Acceleration must be a number")

    if units not in acceleration_lookup:
        raise ValueError("Invalid units for acceleration conversion")

    factor = acceleration_lookup[units]
    return round_sig(acceleration / factor)


def convert_acceleration_from_si(units, acceleration):
    """
    Converts acceleration from other units to metres per square second.

    :param units: One of the units listed in the acceleration lookup dictionary
    :type units: str
    :param acceleration: The acceleration to convert from
    :type acceleration: number
    :return: The converted acceleration
    :rtype: float
    """
    global acceleration_lookup

    if not isinstance(acceleration, numbers.Real):
        raise ValueError("Acceleration must be a number")

    if units not in acceleration_lookup:
        raise ValueError("Invalid units for acceleration conversion")

    factor = acceleration_lookup[units]
    return round_sig(acceleration * factor)


def round_sig(x, sig=4):
    """
    Rounds a number to a number of significant figures.

    :param x: The number to round
    :type x: number
    :param sig: Number of significant figures to round to(defaults to 4)
    :type sig: int
    :return: Rounded number
    :type: float
    """
    if x == 0:
        return 0.0
    return round(x, sig - int(floor(log10(abs(x)))) - 1)
