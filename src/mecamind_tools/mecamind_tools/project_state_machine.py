from dataclasses import dataclass
from enum import Enum


class ProjectMode(str, Enum):
    BOOT = "boot"
    SIM_MAPPING = "sim_mapping"
    SIM_NAVIGATION = "sim_navigation"
    REAL_MAPPING = "real_mapping"
    REAL_NAVIGATION = "real_navigation"
    DEMO = "demo"
    FAULT = "fault"
    ESTOP = "estop"


class ProjectEvent(str, Enum):
    START_SIM = "start_sim"
    START_REAL = "start_real"
    BEGIN_MAPPING = "begin_mapping"
    BEGIN_NAVIGATION = "begin_navigation"
    DEMO_COMPLETE = "demo_complete"
    SENSOR_FAULT = "sensor_fault"
    CLEAR_FAULT = "clear_fault"
    ESTOP = "estop"
    RESET_ESTOP = "reset_estop"
    STOP = "stop"


@dataclass
class TransitionResult:
    previous: ProjectMode
    current: ProjectMode
    accepted: bool
    reason: str


class ProjectStateMachine:
    def __init__(self) -> None:
        self.mode = ProjectMode.BOOT

    def handle(self, event: ProjectEvent) -> TransitionResult:
        previous = self.mode

        if event == ProjectEvent.ESTOP:
            self.mode = ProjectMode.ESTOP
            return TransitionResult(previous, self.mode, True, "estop latched")

        if self.mode == ProjectMode.ESTOP:
            if event == ProjectEvent.RESET_ESTOP:
                self.mode = ProjectMode.BOOT
                return TransitionResult(previous, self.mode, True, "estop reset")
            return TransitionResult(previous, self.mode, False, "estop latched")

        if event == ProjectEvent.SENSOR_FAULT:
            self.mode = ProjectMode.FAULT
            return TransitionResult(previous, self.mode, True, "sensor fault")

        if self.mode == ProjectMode.FAULT:
            if event == ProjectEvent.CLEAR_FAULT:
                self.mode = ProjectMode.BOOT
                return TransitionResult(previous, self.mode, True, "fault cleared")
            return TransitionResult(previous, self.mode, False, "fault requires clear")

        if self.mode == ProjectMode.BOOT:
            if event == ProjectEvent.START_SIM:
                self.mode = ProjectMode.SIM_MAPPING
                return TransitionResult(previous, self.mode, True, "simulation start")
            if event == ProjectEvent.START_REAL:
                self.mode = ProjectMode.REAL_MAPPING
                return TransitionResult(previous, self.mode, True, "real robot start")

        if self.mode in {ProjectMode.SIM_MAPPING, ProjectMode.REAL_MAPPING}:
            if event == ProjectEvent.BEGIN_NAVIGATION:
                self.mode = (
                    ProjectMode.SIM_NAVIGATION
                    if self.mode == ProjectMode.SIM_MAPPING
                    else ProjectMode.REAL_NAVIGATION
                )
                return TransitionResult(previous, self.mode, True, "switch to navigation")
            if event == ProjectEvent.DEMO_COMPLETE:
                self.mode = ProjectMode.DEMO
                return TransitionResult(previous, self.mode, True, "mission done")

        if event == ProjectEvent.STOP:
            self.mode = ProjectMode.BOOT
            return TransitionResult(previous, self.mode, True, "stop and reset")

        return TransitionResult(previous, self.mode, False, "event refused in current mode")
