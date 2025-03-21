from dataclasses import dataclass

@dataclass
class CommandStepper:
    mode: float
    x_des: float
    v_des: float
    a_des: float
    stiffness: float
    i_feedforward: float
    i_contact_pos: float
    i_contact_neg: float
    incr_trigger: float

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)

        return status

    @classmethod
    def init(cls):
        return cls.fromDict(
            {
                "mode": 0,
                "x_des": 0,
                "v_des": 0,
                "a_des": 0,
                "stiffness": 1.0,
                "i_feedforward": 0.0,
                "i_contact_pos": 0,
                "i_contact_neg": 0,
                "incr_trigger": 0,
            }
        )
