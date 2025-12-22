from datetime import datetime
from typing import Optional
from pydantic import BaseModel
from uuid import uuid4


class Message(BaseModel):
    id: str
    session_id: str
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime
    metadata: dict

    def __init__(self, **data):
        super().__init__(**data)
        if 'id' not in data or data['id'] is None:
            self.id = str(uuid4())
        if 'timestamp' not in data or data['timestamp'] is None:
            self.timestamp = datetime.now()