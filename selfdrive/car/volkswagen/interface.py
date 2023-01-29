from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.swaglog import cloudlog
from selfdrive.car.volkswagen.values import CAR, PQ_CARS, BUTTON_STATES, TransmissionType, GearShifter
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.displayMetricUnitsPrev = None
    self.buttonStatesPrev = BUTTON_STATES.copy()

    self.pqCounter = 0
    self.wheelGrabbed = False
    self.pqBypassCounter = 0

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "volkswagen"
    ret.communityFeature = True
    ret.carName = "volkswagen"
    ret.radarOffCan = True

    # Set common PQ35/PQ46/NMS parameters that will apply globally
    ret.safetyModel = car.CarParams.SafetyModel.volkswagenPq
    ret.steerActuatorDelay = 0.05

    if 0x440 in fingerprint[0]:
      # Getriebe_1 detected: traditional automatic or DSG gearbox
      ret.transmissionType = TransmissionType.automatic
    else:
      # No trans message at all, must be a true stick-shift manual
      ret.transmissionType = TransmissionType.manual
    cloudlog.info("Detected transmission type: %s", ret.transmissionType)
    
    # Global tuning defaults, can be overridden per-vehicle

    ret.steerActuatorDelay = 0.05
    ret.steerRateCost = 1.0
    ret.steerLimitTimer = 0.4
    ret.steerRatio = 15.6  # Let the params learner figure this out
    tire_stiffness_factor = 0.8  # Let the params learner figure this out
    # ret.lateralTuning.pid.kf = 0.00006
    
    ret.lateralTuning.pid.kfBP = [20/3.6, 50/3.6, 70/3.6, 90/3.6]
    ret.lateralTuning.pid.kpBP = [0., 105*CV.KPH_TO_MS]
    ret.lateralTuning.pid.kiBP = [0., 105*CV.KPH_TO_MS]
    ret.lateralTuning.pid.kdBP = [0., 105*CV.KPH_TO_MS]

    ret.lateralTuning.pid.kfV = [0.00060, 0.00030, 0.00020, 0.00010]
    ret.lateralTuning.pid.kpV = [0.15, 0.19]
    ret.lateralTuning.pid.kiV = [0.05,  0.08]
    ret.lateralTuning.pid.kdV = [0., 0.]

    ret.mass = 1617+100+STD_CARGO_KG
    ret.wheelbase = 2.68
    ret.minSteerSpeed = 50 * CV.KPH_TO_MS #TEST

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    buttonEvents = []

    # Process the most recent CAN message traffic, and check for validity
    # The camera CAN has no signals we use at this time, but we process it
    # anyway so we can test connectivity with can_valid
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.CP.transmissionType)
    ret.canValid = True
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # TODO: add a field for this to carState, car interface code shouldn't write params
    # Update the device metric configuration to match the car at first startup,
    # or if there's been a change.
    #if self.CS.displayMetricUnits != self.displayMetricUnitsPrev:
    #  put_nonblocking("IsMetric", "1" if self.CS.displayMetricUnits else "0")

    # Check for and process state-change events (button press or release) from
    # the turn stalk switch or ACC steering wheel/control stalk buttons.
    for button in self.CS.buttonStates:
      if self.CS.buttonStates[button] != self.buttonStatesPrev[button]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = button
        be.pressed = self.CS.buttonStates[button]
        buttonEvents.append(be)

    events = self.create_common_events(ret, extra_gears=[GearShifter.eco, GearShifter.sport])

    # if self.CS.leftBlinker:
      # events.add(EventName.buttonEnable)

    #PQTIMEBOMB STUFF START
    #Warning alert for the 6min timebomb found on PQ's
    ret.stopSteering = False
    if True: #(self.frame % 100) == 0: # Set this to false/False if you want to turn this feature OFF!
      self.pqCounter += 1
      if self.pqCounter >= 330*100: #time in seconds until counter threshold for pqTimebombWarn alert
        if not self.wheelGrabbed:
          events.add(EventName.pqTimebombWarn)
          if self.pqCounter >= 345*100: #time in seconds until pqTimebombTERMINAL
            events.add(EventName.pqTimebombTERMINAL)
            if self.pqCounter >= 359*100: #time in seconds until auto bypass
              self.wheelGrabbed = True
        if self.wheelGrabbed or ret.steeringPressed:
          self.wheelGrabbed = True
          ret.stopSteering = True
          self.pqBypassCounter += 1
          if self.pqBypassCounter >= 1.05*100: #time alloted for bypass
            self.wheelGrabbed = False
            self.pqCounter = 0
            self.pqBypassCounter = 0
            events.add(EventName.pqTimebombBypassed)
          else:
            events.add(EventName.pqTimebombBypassing)

    #PQTIMEBOMB STUFF END

    # if self.CS.rightBlinker:
    #   events.add(EventName.espDisabled)
      
    # Vehicle health and operation safety checks
    if self.CS.parkingBrakeSet:
      events.add(EventName.parkBrake)
    if self.CS.steeringFault:
      events.add(EventName.steerTempUnavailable)
    # if ret.vEgo < self.CP.minSteerSpeed:
    #   events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()
    ret.buttonEvents = buttonEvents

    # update previous car states
    self.displayMetricUnitsPrev = self.CS.displayMetricUnits
    self.buttonStatesPrev = self.CS.buttonStates.copy()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, 2, c.actuators,
                   c.hudControl.visualAlert,
                   c.hudControl.leftLaneVisible,
                   c.hudControl.rightLaneVisible,
                   False, 
                   False)
    self.frame += 1
    return can_sends
