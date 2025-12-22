import React, { useState, useEffect, useRef } from 'react';
import Message from './Message';
import './ChatWindow.css';

const ChatWindow = ({ isOpen, onClose, messages, onSendMessage, isLoading }) => {
  const [inputMessage, setInputMessage] = useState('');
  const messagesEndRef = useRef(null);

  const handleSend = () => {
    if (inputMessage.trim() && onSendMessage) {
      onSendMessage(inputMessage);
      setInputMessage('');
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  if (!isOpen) return null;

  return (
    <div className="chat-window">
      <div className="chat-header">
        <span>Documentation Assistant</span>
        <button className="close-button" onClick={onClose}>×</button>
      </div>
      
      <div className="chat-messages">
        {messages.map((msg, index) => (
          <Message 
            key={index} 
            message={msg.content} 
            isUser={msg.role === 'user'} 
          />
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
        <button onClick={handleSend} disabled={!inputMessage.trim() || isLoading}>
          Send
        </button>
      </div>
    </div>
  );
};

export default ChatWindow;