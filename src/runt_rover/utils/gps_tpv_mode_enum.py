#!/usr/bin/env python 
from enum import Enum

class TPV_MODE(Enum):
    """
    This class enumerates the modes that may be present on a TPV (time-position-velocity)
    report. Modes 0,1,2 and 3 are defined as per the protocol in the link below, and -1 
    indicates that the TPV report does not contain a mode attribute.

    See: https://gpsd.gitlab.io/gpsd/gpsd_json.html for further information.
    """

    MODE_UNKNOWN = -1
    UNKNOWN = 0
    NO_FIX = 1
    DIMENSION_2D = 2
    DIMESNION_3D = 3