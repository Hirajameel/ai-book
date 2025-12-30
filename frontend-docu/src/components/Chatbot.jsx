/**
 * Chatbot Component for RAG Agent Integration
 *
 * This component provides a chat interface for users to query the RAG agent.
 */

import React, { useState, useRef } from 'react';
import apiService from '../services/api-service';

const Chatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [contextType, setContextType] = useState('full_book'); // 'full_book' or 'selected_text'
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Handle sending a message
  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) {
      return;
    }

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    // Add user message to the chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Query the RAG agent with appropriate context
      const response = await apiService.queryRAG(inputValue, contextType, contextType === 'selected_text' ? selectedText : null);

      if (response.success && response.data) {
        const botMessage = {
          id: Date.now() + 1,
          text: response.data.answer,
          sender: 'bot',
          sources: response.data.sources || [],
          timestamp: new Date().toISOString()
        };

        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: response.error || 'An error occurred while processing your query.',
          sender: 'bot',
          error: true,
          timestamp: new Date().toISOString()
        };

        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (err) {
      setError(err.message);

      const errorMessage = {
        id: Date.now() + 1,
        text: `Error: ${err.message}`,
        sender: 'bot',
        error: true,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle Enter key press
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Check API health on component mount
  React.useEffect(() => {
    const checkHealth = async () => {
      try {
        await apiService.healthCheck();
        console.log('API is healthy');
      } catch (error) {
        console.error('API health check failed:', error);
      }
    };

    checkHealth();
  }, []);

  // Scroll to bottom when messages change
  React.useEffect(() => {
    scrollToBottom();
  }, [messages]);

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>RAG Agent Chat</h3>
        <p>Ask questions about the book content</p>
      </div>

      <div className="chatbot-context-selector">
        <label>
          Query Context:
          <select
            value={contextType}
            onChange={(e) => setContextType(e.target.value)}
          >
            <option value="full_book">Full Book Content</option>
            <option value="selected_text">Selected Text Only</option>
          </select>
        </label>

        {contextType === 'selected_text' && (
          <div className="selected-text-input">
            <label htmlFor="selected-text">Selected Text:</label>
            <textarea
              id="selected-text"
              value={selectedText}
              onChange={(e) => setSelectedText(e.target.value)}
              placeholder="Enter or paste the text you want to ask about..."
              rows="3"
            />
          </div>
        )}
      </div>

      <div className="chatbot-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your RAG assistant. Ask me anything about the book content.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
            >
              <div className="message-content">
                <p>{message.text}</p>
                {message.sender === 'bot' && message.sources && message.sources.length > 0 && (
                  <div className="sources">
                    <h4>Sources:</h4>
                    <ul>
                      {message.sources.map((source, index) => (
                        <li key={index}>
                          <a href={source.source_id} target="_blank" rel="noopener noreferrer">
                            {source.source_title}
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="loading-message">
            <p>Thinking...</p>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {error && (
        <div className="error-message">
          <p>Error: {error}</p>
        </div>
      )}

      <div className="chatbot-input">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the book content..."
          rows="3"
          disabled={isLoading}
        />
        <button
          onClick={handleSendMessage}
          disabled={!inputValue.trim() || isLoading}
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default Chatbot;