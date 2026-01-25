/**
 * API service to call the backend
 * Handles communication with the RAG agent API
 */

// For Docusaurus, we need to handle the API differently
// Since Docusaurus serves static content, we'll use a proxy or relative path
// For development, we'll keep the localhost URL
// For production, this would need to be configured appropriately
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

class ApiService {
  /**
   * Send a query to the RAG agent
   * @param {Object} queryRequest - The query request object
   * @returns {Promise<Object>} The response from the RAG agent
   */
  static async sendQuery(queryRequest) {
    try {
      const response = await fetch(`${API_BASE_URL}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(queryRequest),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error sending query:', error);
      throw error;
    }
  }

  /**
   * Check the health of the API
   * @returns {Promise<Object>} The health status
   */
  static async checkHealth() {
    try {
      const response = await fetch(`${API_BASE_URL}/health`);
      
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error checking health:', error);
      throw error;
    }
  }
}

export default ApiService;