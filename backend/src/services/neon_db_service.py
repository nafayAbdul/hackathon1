import psycopg2
import psycopg2.extras
import os
from typing import Optional
import logging
from dotenv import load_dotenv

load_dotenv()

logger = logging.getLogger(__name__)

class NeonDBService:
    def __init__(self, connection_url: Optional[str] = None):
        self.connection_url = connection_url or os.getenv("NEON_DB_URL")
        if not self.connection_url:
            raise ValueError("NEON_DB_URL environment variable is required")
        self.connection = None

    def connect(self):
        """Create a connection to the Neon database"""
        try:
            self.connection = psycopg2.connect(
                self.connection_url
            )
            logger.info("Connected to Neon database")

            # Initialize tables if they don't exist
            self._initialize_tables()
        except Exception as e:
            logger.error(f"Failed to connect to Neon database: {e}")
            raise

    def _initialize_tables(self):
        """Create required tables if they don't exist"""
        if not self.connection:
            raise RuntimeError("Database not connected")

        with self.connection.cursor() as cur:
            # Create chat_sessions table
            cur.execute('''
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    id TEXT PRIMARY KEY,
                    user_id TEXT,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    metadata JSON DEFAULT '{}'
                )
            ''')

            # Create messages table
            cur.execute('''
                CREATE TABLE IF NOT EXISTS messages (
                    id TEXT PRIMARY KEY,
                    session_id TEXT REFERENCES chat_sessions(id),
                    role TEXT NOT NULL,
                    content TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    metadata JSON DEFAULT '{}'
                )
            ''')

            # Create users table
            cur.execute('''
                CREATE TABLE IF NOT EXISTS users (
                    id TEXT PRIMARY KEY,
                    email TEXT UNIQUE,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    preferences JSON DEFAULT '{}',
                    metadata JSON DEFAULT '{}'
                )
            ''')

            self.connection.commit()
            logger.info("Database tables initialized")

    def close(self):
        """Close the database connection"""
        if self.connection:
            self.connection.close()

    def get_session(self, session_id: str):
        """Retrieve a chat session by ID"""
        if not self.connection:
            raise RuntimeError("Database not connected")

        with self.connection.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            cur.execute(
                "SELECT * FROM chat_sessions WHERE id = %s",
                (session_id,)
            )
            row = cur.fetchone()
            return row

    def create_session(self, session_id: str, user_id: Optional[str] = None, metadata: dict = None):
        """Create a new chat session"""
        if not self.connection:
            raise RuntimeError("Database not connected")

        if metadata is None:
            metadata = {}

        import json
        metadata_json = json.dumps(metadata)

        with self.connection.cursor() as cur:
            cur.execute('''
                INSERT INTO chat_sessions (id, user_id, metadata)
                VALUES (%s, %s, %s)
            ''', (session_id, user_id, metadata_json))
            self.connection.commit()

    def save_message(self, message_id: str, session_id: str, role: str, content: str, metadata: dict = None):
        """Save a message to the database"""
        if not self.connection:
            raise RuntimeError("Database not connected")

        if metadata is None:
            metadata = {}

        import json
        metadata_json = json.dumps(metadata)

        with self.connection.cursor() as cur:
            cur.execute('''
                INSERT INTO messages (id, session_id, role, content, metadata)
                VALUES (%s, %s, %s, %s, %s)
            ''', (message_id, session_id, role, content, metadata_json))
            self.connection.commit()

    def get_session_messages(self, session_id: str):
        """Retrieve all messages for a session"""
        if not self.connection:
            raise RuntimeError("Database not connected")

        with self.connection.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            cur.execute(
                "SELECT * FROM messages WHERE session_id = %s ORDER BY timestamp ASC",
                (session_id,)
            )
            rows = cur.fetchall()
            return rows

    def update_session_timestamp(self, session_id: str):
        """Update the updated_at timestamp for a session"""
        if not self.connection:
            raise RuntimeError("Database not connected")

        with self.connection.cursor() as cur:
            cur.execute(
                "UPDATE chat_sessions SET updated_at = CURRENT_TIMESTAMP WHERE id = %s",
                (session_id,)
            )
            self.connection.commit()