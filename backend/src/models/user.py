from datetime import datetime
from typing import Optional
from pydantic import BaseModel, EmailStr
from uuid import uuid4


class User(BaseModel):
    id: str
    email: Optional[EmailStr] = None
    created_at: datetime
    preferences: dict
    metadata: dict

    def __init__(self, **data):
        super().__init__(**data)
        if 'id' not in data or data['id'] is None:
            self.id = str(uuid4())
        if 'created_at' not in data or data['created_at'] is None:
            self.created_at = datetime.now()