from datetime import datetime
from typing import Optional
from pydantic import BaseModel
from uuid import uuid4


class UserQuery(BaseModel):
    id: str
    session_id: str
    content: str
    timestamp: datetime
    processed: bool = False
    metadata: dict

    def __init__(self, **data):
        super().__init__(**data)
        if 'id' not in data or data['id'] is None:
            self.id = str(uuid4())
        if 'timestamp' not in data or data['timestamp'] is None:
            self.timestamp = datetime.now()