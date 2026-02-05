import React, { useState } from 'react';
import { apiService } from '../../services/api';

interface SearchResult {
  success: boolean;
  response?: string;
  sources?: Array<{
    url: string;
    section: string;
    chunk_id: string;
    score: number;
  }>;
  metadata?: {
    processing_time: number;
    timestamp: string;
  };
  error?: string;
}

const RAGSearch: React.FC = () => {
  const [query, setQuery] = useState('');
  const [loading, setLoading] = useState(false);
  const [result, setResult] = useState<SearchResult | null>(null);

  const handleSearch = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!query.trim()) return;

    setLoading(true);
    setResult(null);

    try {
      const response = await apiService.query({ query });
      setResult(response as SearchResult);
    } catch (error) {
      setResult({
        success: false,
        error: `An error occurred: ${error instanceof Error ? error.message : 'Unknown error'}`
      });
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="rag-search-container">
      <h2>RAG-Powered Search</h2>
      <form onSubmit={handleSearch} className="search-form">
        <div className="input-group">
          <input
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="Ask anything about Physical AI & Humanoid Robotics..."
            className="search-input"
            disabled={loading}
          />
          <button type="submit" disabled={loading} className="search-button">
            {loading ? 'Searching...' : 'Search'}
          </button>
        </div>
      </form>

      {result && (
        <div className="search-results">
          {result.success ? (
            <>
              <div className="answer-section">
                <h3>Answer:</h3>
                <div className="answer-content">
                  {result.response ? result.response : 'No response available.'}
                </div>
              </div>

              {result.sources && result.sources.length > 0 && (
                <div className="sources-section">
                  <h3>Sources:</h3>
                  <ul className="sources-list">
                    {result.sources.map((source, index) => (
                      <li key={index} className="source-item">
                        <a href={source.url} target="_blank" rel="noopener noreferrer">
                          {source.section || source.url}
                        </a>
                        <span className="confidence">Confidence: {source.score.toFixed(2)}</span>
                      </li>
                    ))}
                  </ul>
                </div>
              )}

              {result.metadata && (
                <div className="metadata-section">
                  <small>Processed in {result.metadata.processing_time.toFixed(2)} seconds</small>
                </div>
              )}
            </>
          ) : (
            <div className="error-section">
              <h3>Error:</h3>
              <p>{result.error || 'An unknown error occurred'}</p>
            </div>
          )}
        </div>
      )}

      <style jsx>{`
        .rag-search-container {
          max-width: 800px;
          margin: 2rem auto;
          padding: 0 1rem;
        }

        .search-form {
          margin-bottom: 2rem;
        }

        .input-group {
          display: flex;
          gap: 0.5rem;
        }

        .search-input {
          flex: 1;
          padding: 0.75rem;
          border: 1px solid #ccc;
          border-radius: 4px;
          font-size: 1rem;
        }

        .search-button {
          padding: 0.75rem 1.5rem;
          background-color: #007cba;
          color: white;
          border: none;
          border-radius: 4px;
          cursor: pointer;
          font-size: 1rem;
        }

        .search-button:disabled {
          background-color: #cccccc;
          cursor: not-allowed;
        }

        .search-results {
          border: 1px solid #e0e0e0;
          border-radius: 8px;
          padding: 1.5rem;
          background-color: #fafafa;
        }

        .answer-section h3 {
          margin-top: 0;
          color: #333;
        }

        .answer-content {
          line-height: 1.6;
          margin-bottom: 1.5rem;
          white-space: pre-wrap;
        }

        .sources-section h3 {
          margin-top: 1.5rem;
          margin-bottom: 0.5rem;
          color: #333;
        }

        .sources-list {
          list-style-type: none;
          padding-left: 0;
        }

        .source-item {
          padding: 0.5rem 0;
          border-bottom: 1px solid #eee;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }

        .source-item:last-child {
          border-bottom: none;
        }

        .source-item a {
          color: #007cba;
          text-decoration: none;
        }

        .source-item a:hover {
          text-decoration: underline;
        }

        .confidence {
          font-size: 0.85rem;
          color: #666;
          background-color: #eef7ff;
          padding: 0.2rem 0.5rem;
          border-radius: 12px;
        }

        .metadata-section {
          margin-top: 1rem;
          color: #666;
          font-style: italic;
        }

        .error-section {
          color: #d32f2f;
          padding: 1rem;
          background-color: #ffebee;
          border-radius: 4px;
        }
      `}</style>
    </div>
  );
};

export default RAGSearch;