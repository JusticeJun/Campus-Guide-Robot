import math

from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS_M = 6371000.0


def valid_coordinates(latitude, longitude):
    return (
        math.isfinite(latitude)
        and math.isfinite(longitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
    )


def valid_nav_sat_fix(msg):
    return (
        isinstance(msg, NavSatFix)
        and msg.status.status >= NavSatStatus.STATUS_FIX
        and valid_coordinates(msg.latitude, msg.longitude)
    )


def haversine_distance(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    a = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad)
        * math.sin(delta_lon / 2.0) ** 2
    )
    return EARTH_RADIUS_M * 2.0 * math.atan2(
        math.sqrt(a), math.sqrt(1.0 - a)
    )
