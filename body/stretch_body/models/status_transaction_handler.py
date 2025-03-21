from dataclasses import dataclass


@dataclass
class StatusTransactionHandler:
    read_error: int
    write_error: int
    transactions: int

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)
    
    @classmethod
    def fromDict(cls, json: dict):
        return cls(**json)

    @classmethod
    def init(cls):
        return cls.fromDict(
            {"read_error": 0, "write_error": 0, "transactions": 0}
        )
