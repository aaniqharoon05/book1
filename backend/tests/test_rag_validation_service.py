import pytest
import os
from unittest.mock import Mock, patch
from src.models.validation_models import TestQuery, RetrievedChunk
from src.services.rag_validation_service import RAGValidationService


def test_rag_validation_service_initialization():
    """Test the initialization of RAGValidationService"""
    # Mock environment variables to avoid needing real API keys
    with patch.dict(os.environ, {
        'QDRANT_HOST': 'localhost',
        'QDRANT_PORT': '6333',
        'QDRANT_COLLECTION_NAME': 'test_collection',
        'COHERE_API_KEY': 'test-key'
    }):
        service = RAGValidationService()
        assert service.collection_name == 'test_collection'


@patch('src.services.rag_validation_service.cohere.Client')
@patch('src.services.rag_validation_service.QdrantClient')
def test_query_vector_db(mock_qdrant_client, mock_cohere_client):
    """Test the query_vector_db method"""
    # Setup mocks
    mock_client_instance = Mock()
    mock_client_instance.embed.return_value = Mock(embeddings=[[0.1, 0.2, 0.3]])
    mock_cohere_client.return_value = mock_client_instance
    
    mock_qdrant_instance = Mock()
    mock_search_result = Mock()
    mock_search_result.id = 'test-id'
    mock_search_result.score = 0.9
    mock_search_result.payload = {'text': 'test text', 'source': 'test source'}
    mock_qdrant_instance.search.return_value = [mock_search_result]
    mock_qdrant_client.return_value = mock_qdrant_instance
    
    # Mock environment variables
    with patch.dict(os.environ, {
        'QDRANT_HOST': 'localhost',
        'QDRANT_PORT': '6333',
        'QDRANT_COLLECTION_NAME': 'test_collection',
        'COHERE_API_KEY': 'test-key'
    }):
        service = RAGValidationService()
        
        # Set qdrant client and cohere client to the mocked instances
        service.qdrant_client = mock_qdrant_instance
        service.cohere_client = mock_client_instance
        
        result = service.query_vector_db("test query")
        
        assert len(result) == 1
        assert result[0].chunk_id == 'test-id'
        assert result[0].text == 'test text'
        assert result[0].similarity_score == 0.9
        assert result[0].source_document == 'test source'


def test_calculate_relevance_score():
    """Test the calculate_relevance_score method"""
    # Mock environment variables
    with patch.dict(os.environ, {
        'QDRANT_HOST': 'localhost',
        'QDRANT_PORT': '6333',
        'QDRANT_COLLECTION_NAME': 'test_collection',
        'COHERE_API_KEY': 'test-key'
    }):
        service = RAGValidationService()
        
        # Using mock for cohere client
        with patch.object(service.cohere_client, 'embed') as mock_embed:
            mock_embed.return_value = Mock(embeddings=[[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]])
            score = service.calculate_relevance_score("query text", "chunk text")
            assert isinstance(score, float)


def test_run_integration_tests():
    """Test the run_integration_tests method"""
    # Mock environment variables
    with patch.dict(os.environ, {
        'QDRANT_HOST': 'localhost',
        'QDRANT_PORT': '6333',
        'QDRANT_COLLECTION_NAME': 'test_collection',
        'COHERE_API_KEY': 'test-key'
    }):
        service = RAGValidationService()
        
        # Mock both services
        with patch.object(service.cohere_client, 'embed') as mock_embed, \
             patch.object(service.qdrant_client, 'get_collections') as mock_get_collections, \
             patch.object(service, 'query_vector_db') as mock_query:
            
            mock_embed.return_value = Mock(embeddings=[[0.1, 0.2, 0.3]])
            mock_get_collections.return_value = Mock()
            mock_query.return_value = []
            
            results = service.run_integration_tests()
            
            # Verify the result structure
            assert 'timestamp' in results
            assert 'tests_passed' in results
            assert 'tests_failed' in results
            assert 'total_tests' in results
            assert 'components_tested' in results
            assert 'failures' in results
            assert 'details' in results