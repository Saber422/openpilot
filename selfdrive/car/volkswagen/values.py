# flake8: noqa

from collections import defaultdict
from typing import Dict

from cereal import car
from selfdrive.car import dbc_dict

Ecu = car.CarParams.Ecu
NetworkLocation = car.CarParams.NetworkLocation
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter

class CarControllerParams:
  HCA_STEP = 2                   # HCA_01 message frequency 50Hz
  MQB_LDW_STEP = 10              # LDW_02 message frequency 10Hz on MQB
  PQ_LDW_STEP = 5                # LDW message frequency 20Hz on PQ35/PQ46/NMS
  GRA_ACC_STEP = 3               # GRA_ACC_01 message frequency 33Hz

  GRA_VBP_STEP = 100             # Send ACC virtual button presses once a second
  GRA_VBP_COUNT = 16             # Send VBP messages for ~0.5s (GRA_ACC_STEP * 16)

  # Observed documented MQB limits: 3.00 Nm max, rate of change 5.00 Nm/sec.
  # Limiting rate-of-change based on real-world testing and Comma's safety
  # requirements for minimum time to lane departure.
  STEER_MAX = 300                # Max heading control assist torque 3.00 Nm
  STEER_DELTA_UP = 4             # Max HCA reached in 1.50s (STEER_MAX / (50Hz * 1.50))
  STEER_DELTA_DOWN = 10          # Min HCA reached in 0.60s (STEER_MAX / (50Hz * 0.60))
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 3    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc

class CANBUS:
  pt = 1
  cam = 2

class DBC_FILES:
  mqb = "vw_golf_mk4"  # Used for all cars with MQB-style CAN messaging

DBC = defaultdict(lambda: dbc_dict(DBC_FILES.mqb, None))  # type: Dict[str, Dict[str, str]]

BUTTON_STATES = {
  "accelCruise": False,
  "decelCruise": False,
  "cancel": False,
  "setCruise": False,
  "resumeCruise": False,
  "gapAdjustCruise": False
}

MQB_LDW_MESSAGES = {
  "none": 0,                            # Nothing to display
  "laneAssistUnavailChime": 1,          # "Lane Assist currently not available." with chime
  "laneAssistUnavailNoSensorChime": 3,  # "Lane Assist not available. No sensor view." with chime
  "laneAssistTakeOverUrgent": 4,        # "Lane Assist: Please Take Over Steering" with urgent beep
  "emergencyAssistUrgent": 6,           # "Emergency Assist: Please Take Over Steering" with urgent beep
  "laneAssistTakeOverChime": 7,         # "Lane Assist: Please Take Over Steering" with chime
  "laneAssistTakeOverSilent": 8,        # "Lane Assist: Please Take Over Steering" silent
  "emergencyAssistChangingLanes": 9,    # "Emergency Assist: Changing lanes..." with urgent beep
  "laneAssistDeactivated": 10,          # "Lane Assist deactivated." silent with persistent icon afterward
}

# Check the 7th and 8th characters of the VIN before adding a new CAR. If the
# chassis code is already listed below, don't add a new CAR, just add to the
# FW_VERSIONS for that existing CAR.

class CAR:
  GOLF_MK6 = "VOLKSWAGEN GOLF 6TH GEN"        # Chassis 1K/5K/AJ, includes Mk6 Golf and variants, or 5th gen with retrofits

# All PQ35/PQ46/NMS platform CARs should be on this list

PQ_CARS = [CAR.GOLF_MK6]

FINGERPRINTS = {
  CAR.GOLF_MK6: [{
    6: 7, 17: 7, 80: 4, 174: 8, 194: 8, 208: 6, 416: 8, 428: 8, 640: 8, 648: 8, 672: 8, 800: 8, 896: 8, 912: 8, 914: 8, 915: 8, 919: 8, 928: 8, 946: 8, 976: 6, 978: 7, 1056: 8, 1152: 8, 1160: 8, 1162: 8, 1164: 8, 1175: 8, 1184: 8, 1192: 8, 1306: 8, 1312: 8, 1344: 8, 1360: 8, 1392: 5, 1394: 1, 1408: 8, 1416: 8, 1420: 8, 1423: 8, 1440: 8, 1463: 8, 1488: 8, 1490: 8, 1494: 5, 1500: 8, 1504: 8, 1512: 8, 1523: 8, 1527: 4, 1656: 4, 1754: 8, 1824: 7, 1827: 7, 2000: 8
  }, {
    0: {}, 1: {174: 8, 416: 8, 672: 8, 1184: 8, 80: 4, 640: 8, 896: 8, 1160: 8, 1344: 8, 208: 6, 194: 8, 919: 8, 428: 8, 928: 8, 1192: 8, 1440: 8, 1394: 1, 648: 8, 1152: 8, 976: 6, 800: 8, 978: 7, 1162: 8, 1416: 8, 912: 8, 914: 8, 915: 8, 1392: 5, 1488: 8, 1504: 8, 1824: 7, 1500: 8, 1164: 8, 1494: 5, 1420: 8, 1312: 8, 1463: 8, 946: 8, 1175: 8, 1360: 8, 1512: 8, 1527: 4, 1423: 8, 1056: 8, 1490: 8, 1827: 7, 1754: 8, 1523: 8, 1306: 8, 2000: 8, 1408: 8}, 2: {}, 3: {}
  }, {
    0: {}, 1: {208: 6, 194: 8, 800: 8, 174: 8, 976: 6, 416: 8, 978: 7, 672: 8, 1184: 8, 80: 4, 640: 8, 648: 8, 896: 8, 1152: 8, 1160: 8, 1162: 8, 1164: 8, 1344: 8, 1416: 8, 428: 8, 928: 8, 1192: 8, 1440: 8, 946: 8, 1312: 8, 1494: 5, 1420: 8, 1394: 1, 1827: 7, 912: 8, 914: 8, 915: 8, 1392: 5, 1488: 8, 1504: 8, 1824: 7, 919: 8, 1175: 8, 1360: 8, 1500: 8, 1463: 8, 1056: 8, 1490: 8, 1512: 8, 1527: 4, 1523: 8, 1423: 8, 1306: 8, 1754: 8, 2000: 8, 1408: 8}, 2: {}, 3: {}
  }],
}

IGNORED_FINGERPRINTS = []
DBC = {
  CAR.GOLF_MK6: dbc_dict('vw_golf_mk4', None),
}
