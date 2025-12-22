import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [inputMessage, setInputMessage] = useState('');
  const messagesEndRef = useRef(null);
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const API_BASE = siteConfig.customFields.apiBaseUrl ||
                  (typeof window !== 'undefined' ?
                   window.location.origin + '/api' :
                   'http://localhost:8000/api');

  // Initialize chat session on component mount
  useEffect(() => {
    const initSession = async () => {
      try {
        // Check if we have a session ID in localStorage
        const storedSessionId = localStorage.getItem('chatSessionId');

        if (storedSessionId) {
          // Use existing session and load its history
          setSessionId(storedSessionId);

          // Load the chat history
          try {
            const response = await fetch(`${API_BASE}/chat/${storedSessionId}/history`);
            const data = await response.json();

            // Format messages for the UI
            const formattedMessages = data.messages.map(msg => ({
              role: msg.role,
              content: msg.content
            }));

            setMessages(formattedMessages);
          } catch (historyError) {
            console.error('Error loading chat history:', historyError);
            // Start a new session if history loading fails
            const newSessionResponse = await fetch(`${API_BASE}/chat/start`, {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
              },
              body: JSON.stringify({}),
            });

            const newSessionData = await newSessionResponse.json();
            setSessionId(newSessionData.session_id);
            localStorage.setItem('chatSessionId', newSessionData.session_id);
          }
        } else {
          // Start a new session
          const response = await fetch(`${API_BASE}/chat/start`, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({}),
          });

          const data = await response.json();
          setSessionId(data.session_id);
          localStorage.setItem('chatSessionId', data.session_id);
        }
      } catch (error) {
        console.error('Error initializing chat session:', error);
      }
    };

    initSession();
  }, []);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputMessage.trim() || !sessionId) {
      return;
    }

    // Add user message to the chat
    const userMessage = { role: 'user', content: inputMessage };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    const messageToSend = inputMessage;
    setInputMessage(''); // Clear input immediately

    try {
      // Get highlighted text if any
      const highlightedText = window.getSelection().toString().trim();

      const response = await fetch(`${API_BASE}/chat/${sessionId}/message`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: messageToSend,
          highlighted_text: highlightedText || null
        }),
      });

      const data = await response.json();

      // Add AI response to the chat
      const aiMessage = {
        role: 'assistant',
        content: data.response,
        sources: data.sources
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request.'
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        <div className="chat-window">
          <div className="chat-header">
            <div className="chat-header-content">
              <img src="/img/Gemini_Generated_Image_d88avid88avid88a-removebg-preview.ico" alt="Physical AI Logo" className="chat-logo" />
              <span>Documentation Assistant</span>
            </div>
            <button className="close-button" onClick={toggleChat}>×</button>
          </div>

          <div className="chat-messages">
            {messages.map((msg, index) => (
              <div key={index} className={`message ${msg.role === 'user' ? 'user-message' : 'ai-message'}`}>
                <div className="message-content">
                  {msg.content}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="message ai-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            <textarea
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the documentation..."
              rows="2"
            />
            <button onClick={sendMessage} disabled={!inputMessage.trim() || isLoading}>
              Send
            </button>
          </div>
        </div>
      ) : null}

      {!isOpen && (
        <button className="chat-toggle-button" onClick={toggleChat}>
          <img src="/img/Gemini_Generated_Image_d88avid88avid88a-removebg-preview.ico" alt="Physical AI Logo" className="chat-toggle-logo" />
          <span>Ask DocuBot</span>
        </button>
      )}
    </div>
  );
};

export default ChatWidget;