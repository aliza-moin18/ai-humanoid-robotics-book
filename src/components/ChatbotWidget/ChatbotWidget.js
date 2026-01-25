import React, { useState } from 'react';
import './ChatbotWidget.css';

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your AI Robotics Assistant. How can I help you today?",
      sender: 'bot',
      timestamp: new Date().toISOString()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = () => setIsOpen(!isOpen);

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const res = await fetch('http://127.0.0.1:8000/query', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: inputValue })
      });

      const data = await res.json();

      const answer =
        data.response ||
        data.answer ||
        'No response from server';

      const botMessage = {
        id: Date.now() + 1,
        text: answer,
        sender: 'bot',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: '⚠️ Server connection failed. Backend not responding.',
        sender: 'system',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chatbot-widget">

      {!isOpen && (
        <button
          className="chatbot-toggle-btn"
          onClick={toggleChat}
          title="Open AI Assistant"
        >
          🤖
        </button>
      )}

      {isOpen && (
        <div className="chatbot-window">

          <div className="chatbot-header">
            <h3> AI Robotics Assistant</h3>
            <button className="close-btn" onClick={toggleChat}>✕</button>
          </div>

          <div className="chatbot-messages">
            {messages.map((msg) => (
              <div key={msg.id} className={`message ${msg.sender}-message`}>
                <div className="message-content">
                  <p>{msg.text}</p>
                </div>
                <div className="message-timestamp">
                  {new Date(msg.timestamp).toLocaleTimeString([], {
                    hour: '2-digit',
                    minute: '2-digit'
                  })}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className="message bot-message">
                <div className="message-content">
                  <p className="loading-text"> Thinking…</p>
                </div>
              </div>
            )}
          </div>

          <form onSubmit={sendMessage} className="chatbot-input-form">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about the book..."
              disabled={isLoading}
            />
            <button
              type="submit"
              disabled={isLoading || !inputValue.trim()}
              title="Send message"
            >
              ➤
            </button>
          </form>

        </div>
      )}
    </div>
  );
};

export default ChatbotWidget;