/**
 * API Service for connecting frontend to the RAG backend
 */

interface QueryRequest {
  query: string;
  options?: Record<string, any>;
}

interface SourceInfo {
  url: string;
  section: string;
  chunk_id: string;
  score: number;
}

interface ResponseMetadata {
  processing_time: number;
  timestamp: string;
  model_used?: string;
  tokens_used?: number;
}

interface QueryResponse {
  success: boolean;
  response: string;
  sources: SourceInfo[];
  metadata: ResponseMetadata;
}

interface ErrorResponse {
  success: boolean;
  error: string;
  error_code: string;
  details: Record<string, any>;
}

class ApiService {
  private baseUrl: string;

  constructor(baseUrl: string = 'http://localhost:8000/api') {
    this.baseUrl = baseUrl;
  }

  /**
   * Sends a query to the RAG backend
   */
  async query(request: QueryRequest): Promise<QueryResponse | ErrorResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error making query:', error);
      return {
        success: false,
        error: `Error making query: ${(error as Error).message}`,
        error_code: 'NETWORK_ERROR',
        details: { query: request.query },
      };
    }
  }

  /**
   * Checks the health of the backend service
   */
  async healthCheck(): Promise<{ status: string; services: Record<string, string> }> {
    try {
      const response = await fetch(`${this.baseUrl}/health`);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error checking health:', error);
      return {
        status: 'error',
        services: { error: (error as Error).message },
      };
    }
  }
}

// Create a singleton instance
const apiService = new ApiService();

export { apiService, ApiService, QueryRequest, QueryResponse, ErrorResponse };