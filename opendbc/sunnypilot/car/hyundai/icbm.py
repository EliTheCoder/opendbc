"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai import hyundaican, hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CANFD_CAR
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# A cruise button used as an *increment* has to be pressed and released, not held.
#
# Measured on a non-SCC Optima across two drives:
#   1 frame per press   -> 1049 frames, zero setpoint changes. CLU11 is 50Hz, so a
#                          single frame asserts the button for ~20ms; too short to latch.
#   25 copies at 10Hz   -> works for sporadic presses, but when the planner wants a
#                          *sustained* change it emits ~250 frames/s against a 50Hz
#                          message. The button is then never released, the BCM sees
#                          one long press and decrements once: 1225 SET_DECEL frames
#                          over 4s left the setpoint pinned while a lead closed.
#
# So emulate a real press: assert the button for PRESS_FRAMES consecutive control
# frames, then stay silent for RELEASE_FRAMES so the car's own CLU11 (button idle)
# is seen on the bus, and only then allow the next press.
PRESS_FRAMES = 15    # 150ms at 100Hz, a human-length press
RELEASE_FRAMES = 10  # 100ms released, so consecutive presses are distinct events

BUTTONS = {
  SendButtonState.increase: Buttons.RES_ACCEL,
  SendButtonState.decrease: Buttons.SET_DECEL,
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.press_frames_left = 0
    self.release_frames_left = 0

  def create_can_mock_button_messages(self, packer, CS, send_button) -> list[CanData]:
    # Mid-release: stay off the bus so the BCM sees the button come back up.
    if self.release_frames_left > 0:
      return []

    # Start a new press only once the previous one has been fully released.
    if self.press_frames_left == 0:
      self.press_frames_left = PRESS_FRAMES

    self.press_frames_left -= 1
    if self.press_frames_left == 0:
      self.release_frames_left = RELEASE_FRAMES

    return [hyundaican.create_clu11(packer, self.frame, CS.clu11, send_button, self.CP)]

  def create_canfd_mock_button_messages(self, packer, CS, CAN, send_button) -> list[CanData]:
    can_sends = []
    if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
      # TODO: resume for alt button cars
      pass
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          for _ in range(20):
            can_sends.append(hyundaicanfd.create_buttons(packer, self.CP, CAN, (CS.buttons_counter + button_counter_offset) % 0xF, send_button))
          self.last_button_frame = self.frame

    return can_sends

  def update(self, CS, CC_SP, packer, frame, last_button_frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    # The release countdown has to advance on every frame, not only while a button
    # is being requested -- otherwise it stalls at idle and delays the next press.
    if self.release_frames_left > 0:
      self.release_frames_left -= 1

    if self.ICBM.sendButton != SendButtonState.none:
      send_button = BUTTONS[self.ICBM.sendButton]

      if self.CP.carFingerprint in CANFD_CAR:
        can_sends.extend(self.create_canfd_mock_button_messages(packer, CS, CAN, send_button))
      else:
        can_sends.extend(self.create_can_mock_button_messages(packer, CS, send_button))

    return can_sends
