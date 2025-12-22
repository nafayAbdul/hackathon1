from fastapi import APIRouter, HTTPException, BackgroundTasks, Request
from typing import Optional
import logging
from uuid import uuid4
import uuid
from datetime import datetime
import os

from src.models.chat_session import ChatSession
from src.models.message import Message
from src.models.user_query import UserQuery
from src.models.generated_response import GeneratedResponse
from src.services.retrieval_service import RetrievalService
from src.services.llm_service import LLMService
from src.services.neon_db_service import NeonDBService
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize services - in a real app, use dependency injection
embedding_service = EmbeddingService()
qdrant_service = QdrantService(
    host=os.getenv("QDRANT_HOST", "localhost"),
    port=int(os.getenv("QDRANT_PORT", 6333)),
    api_key=os.getenv("QDRANT_API_KEY"),
    url=os.getenv("QDRANT_URL")
)
retrieval_service = RetrievalService(qdrant_service, embedding_service)
llm_service = LLMService()
db_service = NeonDBService()

@router.post("/chat/start")
async def start_chat(user_id: Optional[str] = None):
    """Create a new chat session and return a session ID"""
    try:
        # Initialize the database connection if not already done
        if not db_service.connection:
            db_service.connect()

        session_id = str(uuid4())

        # Create the session in the database
        db_service.create_session(session_id, user_id)

        logger.info(f"Started new chat session: {session_id}")
        return {"session_id": session_id}
    except Exception as e:
        logger.error(f"Error starting chat session: {e}")
        raise HTTPException(status_code=500, detail="Failed to start chat session")


import re
from pydantic import BaseModel, Field
from typing import Optional

class MessageRequest(BaseModel):
    content: str = Field(..., min_length=1, max_length=2000, description="The message content")
    highlighted_text: Optional[str] = Field(None, max_length=1000, description="Highlighted text for context")

@router.post("/chat/{session_id}/message")
async def send_message(request: Request, session_id: str):
    """Send a message in a chat session and return the AI-generated response"""
    try:
        # Initialize the database connection if not already done
        if not db_service.connection:
            db_service.connect()

        # Parse request body
        body = await request.json()

        # Validate the request body
        try:
            message_request = MessageRequest(**body)
        except Exception:
            raise HTTPException(status_code=422, detail="Invalid request format")

        content = message_request.content
        highlighted_text = message_request.highlighted_text

        # Additional validation for session_id format (basic check)
        if len(session_id) < 1 or len(session_id) > 128 or not re.match(r'^[a-zA-Z0-9_-]+$', session_id):
            raise HTTPException(status_code=400, detail="Invalid session ID format")

        # Verify the session exists
        session = db_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        # Create a message object for the user's query
        from datetime import datetime
        user_message = Message(
            id=str(uuid.uuid4()),
            session_id=session_id,
            role="user",
            content=content,
            timestamp=datetime.now(),
            metadata={}
        )

        # Save the user's message to the database
        db_service.save_message(
            user_message.id,
            user_message.session_id,
            user_message.role,
            user_message.content,
            user_message.metadata
        )

        # Update the session's timestamp after saving user message
        db_service.update_session_timestamp(session_id)

        # Retrieve relevant documentation, prioritizing content related to highlighted text
        relevant_docs = retrieval_service.retrieve_with_highlighted_text(content, highlighted_text)

        # Generate a response using the LLM with the retrieved context
        response_data = llm_service.generate_response_with_sources(content, relevant_docs)

        # Create a message object for the AI's response
        ai_message = Message(
            id=str(uuid.uuid4()),
            session_id=session_id,
            role="assistant",
            content=response_data["response"],
            timestamp=datetime.now(),
            metadata={}
        )

        # Save the AI's response to the database
        db_service.save_message(
            ai_message.id,
            ai_message.session_id,
            ai_message.role,
            ai_message.content,
            ai_message.metadata
        )

        # Update the session's timestamp again after saving AI response
        db_service.update_session_timestamp(session_id)

        logger.info(f"Processed message for session {session_id}")
        return {
            "response": response_data["response"],
            "sources": response_data["sources"]
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing message: {e}")
        raise HTTPException(status_code=500, detail="Failed to process message")


@router.get("/chat/{session_id}/history")
async def get_chat_history(session_id: str):
    """Retrieve the message history for a specific chat session"""
    try:
        # Initialize the database connection if not already done
        if not db_service.connection:
            db_service.connect()

        # Verify the session exists
        session = db_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        # Get all messages for this session
        messages = db_service.get_session_messages(session_id)

        # Format the messages for response
        formatted_messages = []
        for msg in messages:
            formatted_messages.append({
                "id": msg['id'],
                "role": msg['role'],
                "content": msg['content'],
                "timestamp": msg['timestamp'],
            })

        logger.info(f"Retrieved history for session {session_id} ({len(formatted_messages)} messages)")
        return {"messages": formatted_messages}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error retrieving chat history: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve chat history")