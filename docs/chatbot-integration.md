# Integrating the RAG Chatbot with Docusaurus

This document provides instructions for integrating the RAG Chatbot into a Docusaurus documentation site.

## Overview

The RAG (Retrieval-Augmented Generation) Chatbot allows users to ask questions about your documentation and receive AI-generated answers based on your content. The chat widget appears as a floating button in the bottom-right corner of the screen.

## Prerequisites

- A Docusaurus documentation site
- A backend server running the RAG Chatbot API
- Google Gemini API key
- Qdrant vector database
- Neon PostgreSQL database

## Backend Setup

1. Ensure your backend server is running and accessible from your Docusaurus site
2. Configure the required environment variables:
   - `GEMINI_API_KEY`: Your Google Gemini API key
   - `QDRANT_URL`: URL to your Qdrant instance
   - `QDRANT_API_KEY`: API key for Qdrant
   - `NEON_DB_URL`: Connection string for Neon PostgreSQL database

## Frontend Integration

### Option 1: Using the Plugin (Recommended)

1. Copy the chat widget components to your Docusaurus site:
   - `src/components/ChatWidget/ChatWidget.tsx`
   - `src/components/ChatWidget/ChatWindow.tsx`
   - `src/components/ChatWidget/Message.tsx`
   - Associated CSS files

2. Update the API base URL in the ChatWidget component to point to your backend:
   ```javascript
   const API_BASE_URL = 'https://your-backend-domain.com/api';
   ```

3. Add the ChatWidget component to your site by modifying your layout or using a plugin.

### Option 2: Direct Component Usage

1. Add the ChatWidget component to your site's layout, for example in `src/pages/Layout.js` or by modifying the theme.

## Indexing Your Documentation

Before users can ask questions, you need to index your documentation content:

1. Run the ingestion script:
   ```bash
   cd backend
   python ingest.py --source-url https://your-docs-url.com
   ```

2. Alternatively, call the ingestion API endpoint:
   ```bash
   curl -X POST http://your-backend/api/ingest \
     -H "Content-Type: application/json" \
     -d '{"source_url": "https://your-docs-url.com", "force_reindex": false}'
   ```

## Configuration

### Environment Variables

In your frontend environment, set:
- `REACT_APP_API_URL`: The URL to your backend API (e.g., `https://your-backend-domain.com/api`)

### Customization

You can customize the chat widget's appearance by modifying the CSS files in `src/components/ChatWidget/`.

## Security Considerations

- Ensure your API endpoints are protected against unauthorized access
- Implement proper authentication if needed
- Use HTTPS for all API communications
- Regularly rotate your API keys

## Troubleshooting

### Common Issues

1. **Chat widget not appearing**: Check that the component is properly added to your layout
2. **API connection errors**: Verify that the backend URL is correct and accessible
3. **No responses from bot**: Ensure the documentation has been properly indexed

### Debugging

Enable browser console logging to see API requests and responses for debugging purposes.