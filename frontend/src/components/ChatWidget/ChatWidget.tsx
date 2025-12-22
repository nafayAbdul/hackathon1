import React, { useState, useEffect } from 'react';
import ChatWindow from './ChatWindow';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);

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
            const response = await fetch(`http://localhost:8000/api/chat/${storedSessionId}/history`);
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
            const newSessionResponse = await fetch('http://localhost:8000/api/chat/start', {
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
          const response = await fetch('http://localhost:8000/api/chat/start', {
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

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Function to get selected text
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection.toString().trim();
  };

  const sendMessage = async (content) => {
    if (!sessionId) {
      console.error('No session ID available');
      return;
    }

    // Add user message to the chat
    const userMessage = { role: 'user', content };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Get highlighted text if any
      const highlightedText = getSelectedText();

      const response = await fetch(`http://localhost:8000/api/chat/${sessionId}/message`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content,
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

  return (
    <div className="chat-widget">
      <ChatWindow 
        isOpen={isOpen}
        onClose={() => setIsOpen(false)}
        messages={messages}
        onSendMessage={sendMessage}
        isLoading={isLoading}
      />
      
      {!isOpen && (
        <button className="chat-toggle-button" onClick={toggleChat}>
          <span>Ask DocuBot</span>
        </button>
      )}
    </div>
  );
};

export default ChatWidget;