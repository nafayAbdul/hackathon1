import React from 'react';
import './Message.css';

const Message = ({ message, isUser }) => {
  return (
    <div className={`message ${isUser ? 'user-message' : 'ai-message'}`}>
      <div className="message-content">
        {message}
      </div>
      <div className="message-timestamp">
        {new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
      </div>
    </div>
  );
};

export default Message;