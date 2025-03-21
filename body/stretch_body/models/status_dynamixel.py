from dataclasses import dataclass

from body.stretch_body.dynamixel_hello_XL430 import DynamixelCommErrorStats


@dataclass
class StatusDynamixel:

    timestamp_pc: float
    comm_errors: DynamixelCommErrorStats
    pos: float
    vel: float
    effort: float
    temp: float
    shutdown: int
    hardware_error: int
    input_voltage_error: int
    overheating_error: int
    motor_encoder_error: int
    electrical_shock_error: int
    overload_error: int
    stalled: int
    stall_overload: int
    pos_ticks: int
    vel_ticks: int
    effort_ticks: int
    watchdog_errors: int

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)

        if not isinstance(status.comm_errors,DynamixelCommErrorStats):
            status.comm_errors = DynamixelCommErrorStats(**status.comm_errors)

        return status

    @classmethod
    def init(cls, comm_errors: DynamixelCommErrorStats):
        return cls.fromDict(
            {
                "timestamp_pc": 0,
                "comm_errors": comm_errors,
                "pos": 0,
                "vel": 0,
                "effort": 0,
                "temp": 0,
                "shutdown": 0,
                "hardware_error": 0,
                "input_voltage_error": 0,
                "overheating_error": 0,
                "motor_encoder_error": 0,
                "electrical_shock_error": 0,
                "overload_error": 0,
                "stalled": 0,
                "stall_overload": 0,
                "pos_ticks": 0,
                "vel_ticks": 0,
                "effort_ticks": 0,
                "watchdog_errors": 0,
            }
        )
