from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel
from uuid import uuid4


class DocumentationChunk(BaseModel):
    id: str
    content: str
    source_url: str
    source_title: str
    embedding: List[float]  # Vector representation of the content
    created_at: datetime
    metadata: dict

    def __init__(self, **data):
        super().__init__(**data)
        if 'id' not in data or data['id'] is None:
            self.id = str(uuid4())
        if 'created_at' not in data or data['created_at'] is None:
            self.created_at = datetime.now()