/**
 * API Service for RAG Agent Integration
 *
 * This service handles communication with the FastAPI backend for RAG queries.
 */

class ApiService {
  constructor() {
    // In development, this might point to localhost:8000
    // In production, this would be the deployed backend URL
    // For Docusaurus, we'll use a global variable that can be configured
    this.baseUrl = typeof window !== 'undefined' && window.API_BASE_URL
      ? window.API_BASE_URL
      : 'http://localhost:8000';
  }

  /**
   * Query the RAG agent
   * @param {string} query - The user's query
   * @param {string} contextType - Either 'full_book' or 'selected_text'
   * @param {string} [selectedText] - Text content for context-restricted queries
   * @returns {Promise<Object>} The API response
   */
  async queryRAG(query, contextType = 'full_book', selectedText = null) {
    try {
      const requestBody = {
        query: query,
        context_type: contextType
      };

      if (contextType === 'selected_text' && selectedText) {
        requestBody.selected_text = selectedText;
      }

      const response = await fetch(`${this.baseUrl}/api/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error querying RAG agent:', error);
      throw error;
    }
  }

  /**
   * Check API health
   * @returns {Promise<Object>} The health check response
   */
  async healthCheck() {
    try {
      const response = await fetch(`${this.baseUrl}/api/health`);

      if (!response.ok) {
        throw new Error(`Health check failed with status ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error checking API health:', error);
      throw error;
    }
  }

  /**
   * Set a new base URL for the API
   * @param {string} url - The new base URL
   */
  setBaseUrl(url) {
    this.baseUrl = url;
  }
}

// Create a singleton instance
const apiService = new ApiService();
export default apiService;