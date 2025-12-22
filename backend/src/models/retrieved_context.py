from typing import Optional
from pydantic import BaseModel
from uuid import uuid4


class RetrievedContext(BaseModel):
    id: str
    query_id: str
    chunk_id: str
    relevance_score: float  # Between 0 and 1
    content: str
    metadata: dict

    def __init__(self, **data):
        super().__init__(**data)
        if 'id' not in data or data['id'] is None:
            self.id = str(uuid4())