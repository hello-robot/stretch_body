from dataclasses import asdict, dataclass

from body.stretch_body.models.status_transport import StatusTransport
from body.stretch_body.models.status_waypoint_trajectory import StatusWaypointTrajectory


@dataclass
class StatusStepper:
    mode: int
    effort_ticks: float
    effort_pct: float
    current: float
    pos: float
    vel: float
    err: float
    diag: int
    timestamp: float
    debug: int
    guarded_event: int
    transport: StatusTransport
    pos_calibrated: float
    runstop_on: int
    near_pos_setpoint: float
    near_vel_setpoint: float
    is_moving: bool
    is_moving_filtered: int
    at_current_limit: int
    is_mg_accelerating: int
    is_mg_moving: int
    calibration_rcvd: int
    in_guarded_event: int
    in_safety_event: int
    waiting_on_sync: int
    in_sync_mode: int
    trace_on: int
    ctrl_cycle_cnt: int
    waypoint_traj: StatusWaypointTrajectory
    voltage: float

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)
        # Make waypoint_traj an instance of WaypointTrajectoryStatus:
        if not isinstance(status.waypoint_traj, StatusWaypointTrajectory):
            status.waypoint_traj = StatusWaypointTrajectory.fromDict(
                json["waypoint_traj"]
            )
        if not isinstance(status.transport, StatusTransport):
            status.transport = StatusTransport.fromDict(json["transport"])

        return status

    def toDict(self):
        return asdict(self)

    def copy(self):
        return StatusStepper.fromDict(self.toDict())

    @classmethod
    def init(cls, transport: StatusTransport):
        return cls.fromDict(
            {
                "mode": 0,
                "effort_ticks": 0,
                "effort_pct": 0,
                "current": 0,
                "pos": 0,
                "vel": 0,
                "err": 0,
                "diag": 0,
                "timestamp": 0,
                "debug": 0,
                "guarded_event": 0,
                "transport": transport,
                "pos_calibrated": 0,
                "runstop_on": 0,
                "near_pos_setpoint": 0,
                "near_vel_setpoint": 0,
                "is_moving": 0,
                "is_moving_filtered": 0,
                "at_current_limit": 0,
                "is_mg_accelerating": 0,
                "is_mg_moving": 0,
                "calibration_rcvd": 0,
                "in_guarded_event": 0,
                "in_safety_event": 0,
                "waiting_on_sync": 0,
                "in_sync_mode": 0,
                "trace_on": 0,
                "ctrl_cycle_cnt": 0,
                "waypoint_traj": StatusWaypointTrajectory.init(),
                "voltage": 0,
            }
        )
