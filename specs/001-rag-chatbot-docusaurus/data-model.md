# Data Model: RAG Chatbot for Docusaurus

**Feature**: 001-rag-chatbot-docusaurus  
**Date**: 2025-12-21

## Entity: ChatSession

Represents a user's ongoing conversation with the chatbot, including history of messages and session metadata.

**Fields**:
- `id`: Unique identifier for the session (UUID/string)
- `user_id`: Identifier for the user (optional, for registered users) 
- `created_at`: Timestamp when the session was created (datetime)
- `updated_at`: Timestamp when the session was last updated (datetime)
- `messages`: List of messages in the conversation (array of Message objects)
- `metadata`: Additional session metadata (JSON object)

**Relationships**:
- Contains multiple `Message` entities
- Associated with a `User` (optional)

**Validation Rules**:
- `id` must be unique
- `created_at` must be before `updated_at`
- `messages` array must not exceed size limit (e.g., 50 messages)

## Entity: Message

Represents a single message in a chat session (either user query or system response).

**Fields**:
- `id`: Unique identifier for the message (UUID/string)
- `session_id`: Reference to the parent ChatSession (foreign key)
- `role`: Role of the message sender (string: "user" or "assistant")
- `content`: The actual message content (string)
- `timestamp`: When the message was created (datetime)
- `metadata`: Additional message metadata (JSON object)

**Relationships**:
- Belongs to one `ChatSession`
- May reference multiple `RetrievedContext` entities (for assistant messages)

**Validation Rules**:
- `role` must be either "user" or "assistant"
- `content` must not be empty
- `timestamp` must be within the session timeframe

## Entity: DocumentationChunk

Represents a segment of documentation content that has been processed and stored as vector embeddings for retrieval.

**Fields**:
- `id`: Unique identifier for the chunk (UUID/string)
- `content`: The text content of the chunk (string)
- `source_url`: URL or path to the original documentation page (string)
- `source_title`: Title of the original documentation page (string)
- `embedding`: Vector representation of the content (array of floats)
- `created_at`: When the chunk was created (datetime)
- `metadata`: Additional metadata about the chunk (JSON object)

**Relationships**:
- May be referenced by multiple `RetrievedContext` entities

**Validation Rules**:
- `content` must not be empty
- `source_url` must be a valid URL or path
- `embedding` must have the correct dimensions for the model used

## Entity: UserQuery

Represents a user's input question along with associated metadata like timestamp and session ID.

**Fields**:
- `id`: Unique identifier for the query (UUID/string)
- `session_id`: Reference to the ChatSession (foreign key)
- `content`: The actual query text (string)
- `timestamp`: When the query was submitted (datetime)
- `processed`: Whether the query has been processed (boolean)
- `metadata`: Additional query metadata (JSON object)

**Relationships**:
- Belongs to one `ChatSession`
- May generate multiple `RetrievedContext` entities
- May generate one `GeneratedResponse`

**Validation Rules**:
- `content` must not be empty
- `timestamp` must be within the session timeframe

## Entity: RetrievedContext

Represents documentation segments retrieved by the system to answer a specific user query.

**Fields**:
- `id`: Unique identifier for the retrieved context (UUID/string)
- `query_id`: Reference to the UserQuery (foreign key)
- `chunk_id`: Reference to the DocumentationChunk (foreign key)
- `relevance_score`: Score indicating how relevant the chunk is to the query (float)
- `content`: The relevant content snippet (string)
- `metadata`: Additional metadata about the retrieval (JSON object)

**Relationships**:
- Belongs to one `UserQuery`
- References one `DocumentationChunk`

**Validation Rules**:
- `relevance_score` must be between 0 and 1
- `content` must not be empty

## Entity: GeneratedResponse

Represents the AI-generated answer provided to the user based on the retrieved context.

**Fields**:
- `id`: Unique identifier for the response (UUID/string)
- `query_id`: Reference to the UserQuery (foreign key)
- `content`: The generated response text (string)
- `timestamp`: When the response was generated (datetime)
- `source_chunks`: List of IDs of the chunks used to generate the response (array of strings)
- `metadata`: Additional response metadata (JSON object)

**Relationships**:
- Belongs to one `UserQuery`
- References multiple `RetrievedContext` entities (via source_chunks)

**Validation Rules**:
- `content` must not be empty
- `timestamp` must be after the query's timestamp

## Entity: User

Represents a registered user of the system (optional, for enhanced functionality).

**Fields**:
- `id`: Unique identifier for the user (UUID/string)
- `email`: User's email address (string)
- `created_at`: When the user account was created (datetime)
- `preferences`: User preferences (JSON object)
- `metadata`: Additional user metadata (JSON object)

**Relationships**:
- May have multiple `ChatSession` entities

**Validation Rules**:
- `email` must be a valid email address
- `id` must be unique