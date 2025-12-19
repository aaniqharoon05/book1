import os
import logging
import time
import json
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
import numpy as np
from pydantic import BaseModel
from datetime import datetime
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Qdrant Configuration
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "document_chunks")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Cohere Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

# Application Configuration
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")


class RetrievedChunk(BaseModel):
    """Model for a document chunk retrieved from the vector database"""
    chunk_id: str
    text: str
    similarity_score: float
    source_document: str
    metadata: Optional[dict] = None


class RetrieveService:
    """
    Service class for data retrieval operations from the RAG system
    """

    def __init__(self):
        # Set up logging FIRST
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Initialize Qdrant client (cloud-first, fallback to local)
        if QDRANT_URL:
            self.logger.info("Using Qdrant Cloud")
            self.qdrant_client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
            )
        else:
            self.logger.info("Using local Qdrant")
            self.qdrant_client = QdrantClient(
                host=QDRANT_HOST,
                port=QDRANT_PORT,
            )

        # Initialize Cohere client for embeddings
        self.cohere_client = cohere.Client(COHERE_API_KEY)

        # Collection name for document chunks
        self.collection_name = QDRANT_COLLECTION_NAME

    def query_vector_db(self, query_text: str, top_k: int = 10, similarity_threshold: float = 0.5) -> List[RetrievedChunk]:
        """
        Main function to retrieve document chunks from the vector database based on semantic similarity

        Args:
            query_text: The text to search for in the vector database
            top_k: Maximum number of results to return
            similarity_threshold: Minimum similarity score for results

        Returns:
            List of RetrievedChunk objects containing the most relevant document chunks
        """
        self.logger.info(f"Starting vector database query for: '{query_text[:50]}...'")
        query_start_time = time.time()

        try:
            response = self.cohere_client.embed(
                texts=[query_text],
                model="embed-multilingual-v3.0",  # Updated to multilingual model
                input_type="search_query",
            )
            query_embedding = response.embeddings[0]

            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=similarity_threshold,  # Will filter after retrieval
            )

            points = search_results
            self.logger.debug(f"Retrieved {len(points)} results from vector database")

            retrieved_chunks = []
            for i, result in enumerate(points):
                # Apply similarity threshold filtering
                if result.score >= similarity_threshold:
                    retrieved_chunks.append(
                        RetrievedChunk(
                            chunk_id=result.id,
                            text=result.payload.get("text", ""),
                            similarity_score=result.score,
                            source_document=result.payload.get("source", ""),
                            metadata=result.payload,
                        )
                    )
                else:
                    self.logger.debug(f"Filtered out chunk {result.id} due to low similarity score: {result.score}")

            self.logger.info(
                f"Vector database query completed in {time.time() - query_start_time:.3f}s. "
                f"Retrieved {len(retrieved_chunks)} chunks after filtering."
            )
            return retrieved_chunks

        except Exception as e:
            self.logger.error(f"Error querying vector database: {str(e)}")
            raise

    def calculate_relevance_score(self, query_text: str, chunk_text: str) -> float:
        """
        Calculate relevance score between query and chunk using embedding similarity
        """
        self.logger.debug(f"Calculating relevance score between query and chunk (first 30 chars: '{chunk_text[:30]}...')")
        calc_start_time = time.time()

        try:
            # Generate embeddings for both query and chunk
            response = self.cohere_client.embed(
                texts=[query_text, chunk_text],
                model='embed-multilingual-v3.0',  # Updated to multilingual model
                input_type="search_document"
            )

            query_embedding = response.embeddings[0]
            chunk_embedding = response.embeddings[1]

            # Calculate cosine similarity
            query_norm = np.linalg.norm(query_embedding)
            chunk_norm = np.linalg.norm(chunk_embedding)

            if query_norm == 0 or chunk_norm == 0:
                calc_time = time.time() - calc_start_time
                self.logger.warning(f"Zero norm detected in relevance calculation, returning 0.0 after {calc_time:.3f}s")
                return 0.0

            similarity = np.dot(query_embedding, chunk_embedding) / (query_norm * chunk_norm)
            calc_time = time.time() - calc_start_time
            self.logger.debug(f"Relevance score calculated as {similarity:.3f} in {calc_time:.3f}s")
            return float(similarity)

        except Exception as e:
            calc_time = time.time() - calc_start_time
            self.logger.error(f"Error calculating relevance score after {calc_time:.3f}s: {str(e)}")
            raise

    def verify_chunk(self, chunk: RetrievedChunk) -> bool:
        """
        Verify that a retrieved chunk has valid content and source information.

        Args:
            chunk: The chunk to verify

        Returns:
            Boolean indicating whether the chunk is valid
        """
        is_valid = True

        if not chunk.text:
            self.logger.warning(f"Chunk {chunk.chunk_id} has no content")
            is_valid = False

        if not chunk.source_document:
            self.logger.warning(f"Chunk {chunk.chunk_id} has no source document")
            is_valid = False

        return is_valid

    def format_as_json(self, query: str, retrieved_chunks: List[RetrievedChunk], query_time_ms: float) -> str:
        """
        Format retrieval results as a JSON string according to Code-1 requirements.

        Args:
            query: The original query text
            retrieved_chunks: List of retrieved chunks
            query_time_ms: Time taken for the query in milliseconds

        Returns:
            JSON string with the specified format
        """
        results = []
        for chunk in retrieved_chunks:
            # Verify the chunk before adding it to results
            if self.verify_chunk(chunk):
                result_item = {
                    "content": chunk.text,
                    "url": chunk.source_document,
                    "similarity_score": chunk.similarity_score,
                    "chunk_id": chunk.chunk_id,
                    "metadata": chunk.metadata or {}
                }
                results.append(result_item)

        output = {
            "query": query,
            "results": results,
            "metadata": {
                "query_time_ms": query_time_ms,
                "total_results": len(results),
                "timestamp": datetime.now().isoformat(),
                "collection_name": self.collection_name
            }
        }

        return json.dumps(output, indent=2)

    def retrieve(self, query_text: str, top_k: int = 10, similarity_threshold: float = 0.5) -> str:
        """
        Main retrieval entrypoint for use by agent tools.
        Measures query time, validates results, and returns a JSON string.

        Args:
            query_text: The text to search for in the vector database
            top_k: Maximum number of results to return
            similarity_threshold: Minimum similarity score for results

        Returns:
            JSON string formatted according to Code-1 requirements
        """
        start_time = time.time()

        # Call existing vector search logic
        retrieved_chunks = self.query_vector_db(query_text, top_k, similarity_threshold)

        # Calculate query time in milliseconds
        query_time_ms = (time.time() - start_time) * 1000

        # Perform lightweight validation - filter out invalid chunks
        valid_chunks = [chunk for chunk in retrieved_chunks if self.verify_chunk(chunk)]

        if len(valid_chunks) != len(retrieved_chunks):
            self.logger.warning(f"Filtered out {len(retrieved_chunks) - len(valid_chunks)} invalid chunks")

        # Return results as a JSON string
        return self.format_as_json(query_text, valid_chunks, query_time_ms)


# Function to get an instance of the retrieval service
def get_retrieve_service():
    """
    Creates and returns an instance of the RetrieveService
    """
    return RetrieveService()


# Convenience function for direct retrieval
def retrieve_data(query_text: str, top_k: int = 10, similarity_threshold: float = 0.5) -> List[RetrievedChunk]:
    """
    Convenience function to retrieve data directly

    Args:
        query_text: The text to search for in the vector database
        top_k: Maximum number of results to return
        similarity_threshold: Minimum similarity score for results

    Returns:
        List of RetrievedChunk objects containing the most relevant document chunks
    """
    service = get_retrieve_service()
    return service.query_vector_db(query_text, top_k, similarity_threshold)


def retrieve_all_data(service: RetrieveService = None):
    """
    Debugging utility to scroll through the entire Qdrant collection.
    Prints chunk ID, source URL, position (if available), and content preview.

    Args:
        service: Instance of RetrieveService. If None, creates a new instance.
    """
    if service is None:
        service = get_retrieve_service()

    offset = 0
    batch_size = 100
    retrieved_count = 0

    print("Scrolling entire Qdrant collection...")

    while True:
        # Scroll through the collection in batches
        response = service.qdrant_client.scroll(
            collection_name=service.collection_name,
            offset=offset,
            limit=batch_size,
            with_payload=True,
            with_vectors=False
        )

        points = response.points
        if not points:
            break  # No more points to retrieve

        for point in points:
            # Extract information from the point
            chunk_id = point.id
            payload = point.payload
            source_url = payload.get("source_url", "N/A")
            chunk_position = payload.get("chunk_position", "N/A")
            content_preview = payload.get("text", "")[:200]  # First 200 chars

            print(f"Chunk ID: {chunk_id}")
            print(f"Source URL: {source_url}")
            print(f"Position: {chunk_position}")
            print(f"Content Preview: {content_preview}...")
            print("-" * 50)

            retrieved_count += 1

        offset += len(points)

        # If we got fewer than batch_size points, we're at the end
        if len(points) < batch_size:
            break

    print(f"Scrolled through {retrieved_count} total points in the collection")


if __name__ == "__main__":
    # Example usage - changed to query about Vision-Language-Action (VLA) to match collection content
    # query = "what is Vision-Language-Action (VLA)"
    # results = retrieve_data(query, top_k=5, similarity_threshold=0.05)  # Lowered threshold for better matching

    # print(f"Retrieved {len(results)} chunks for query: '{query}'")
    # for i, chunk in enumerate(results):
    #     print(f"\nChunk {i+1} (Score: {chunk.similarity_score:.3f}):")
    #     print(f"Source: {chunk.source_document}")
    #     # Handle potential Unicode characters in text preview by removing problematic ones
    #     text_preview = chunk.text[:200]
    #     # Remove zero-width space and other problematic characters
    #     import re
    #     clean_text = re.sub(r'[\u200b-\u200d\ufeff]', '', text_preview)
    #     # Replace remaining non-printable characters with a space
    #     clean_text = ''.join(char if ord(char) < 127 and char.isprintable() else ' ' for char in clean_text)
    #     print(f"Text Preview: {clean_text}...")

    query = "what is Vision-Language-Action (VLA)"
    results = retrieve_data(query, top_k=5, similarity_threshold=0.05)

    print(f"Retrieved {len(results)} chunks for query: '{query}'")

    for i, chunk in enumerate(results):
        print(f"\nChunk {i+1} (Score: {chunk.similarity_score:.3f}):")
        print(f"Source: {chunk.source_document}")
        print("Full Text:")
        print(chunk.text)
        print("-" * 80)
        
        with open("full_chunks.txt", "w", encoding="utf-8") as f:
            for i, chunk in enumerate(results):
                f.write(f"\n--- Chunk {i+1} (Score: {chunk.similarity_score:.3f}) ---\n")
                f.write(chunk.text)
                f.write("\n")
