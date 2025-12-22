from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel
from uuid import uuid4


class GeneratedResponse(BaseModel):
    id: str
    query_id: str
    content: str
    timestamp: datetime
    source_chunks: List[str]  # List of IDs of the chunks used to generate the response
    metadata: dict

    def __init__(self, **data):
        super().__init__(**data)
        if 'id' not in data or data['id'] is None:
            self.id = str(uuid4())
        if 'timestamp' not in data or data['timestamp'] is None:
            self.timestamp = datetime.now()