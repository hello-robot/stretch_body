from dataclasses import dataclass

from body.stretch_body.models.status_transaction_handler import StatusTransactionHandler


@dataclass
class StatusTransport:
    asyncHandler: StatusTransactionHandler
    syncHandler: StatusTransactionHandler

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def init(cls):
        return cls(
            asyncHandler=StatusTransactionHandler.init(),
            syncHandler=StatusTransactionHandler.init(),
        )

    @classmethod
    def fromDict(cls, json: dict):
        asyncHandler = json["asyncHandler"]
        syncHandler = json["syncHandler"]

        return cls(
            asyncHandler=(
                StatusTransactionHandler.fromDict(asyncHandler)
                if not isinstance(asyncHandler, StatusTransactionHandler)
                else asyncHandler
            ),
            syncHandler=(
                StatusTransactionHandler.fromDict(syncHandler)
                if not isinstance(syncHandler, StatusTransactionHandler)
                else syncHandler
            ),
        )
