from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel
from uuid import uuid4


class ChatSession(BaseModel):
    id: str
    user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    metadata: dict

    def __init__(self, **data):
        super().__init__(**data)
        if 'id' not in data or data['id'] is None:
            self.id = str(uuid4())
        if 'created_at' not in data or data['created_at'] is None:
            self.created_at = datetime.now()
        if 'updated_at' not in data or data['updated_at'] is None:
            self.updated_at = datetime.now()