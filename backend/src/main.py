"""
Main ingestion pipeline implementation.

This script orchestrates the entire process of:
1. Fetching URLs from the sitemap
2. Extracting and cleaning text content
3. Chunking the text
4. Generating embeddings
5. Storing in Qdrant
"""
import logging
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from typing import List
import requests
from bs4 import BeautifulSoup
import re

from config import settings
from utils import parse_sitemap

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

import time
from typing import Optional

def get_all_urls():
    """
    Get all URLs from the sitemap, with fallback if sitemap doesn't exist.

    Returns:
        List of URLs from the sitemap or a default list if sitemap is unavailable
    """
    try:
        logger.info(f"Fetching URLs from sitemap: {settings.SITEMAP_URL}")
        urls = parse_sitemap(settings.SITEMAP_URL)

        # Fix URLs if they contain the placeholder domain
        fixed_urls = []
        for url in urls:
            fixed_url = url.replace("https://your-docusaurus-site.example.com", settings.SOURCE_URL.rstrip('/'))
            fixed_urls.append(fixed_url)

        logger.info(f"Retrieved {len(fixed_urls)} URLs from sitemap (fixed domains)")
        return fixed_urls
    except Exception as e:
        logger.warning(f"Sitemap not available at {settings.SITEMAP_URL}: {e}")
        logger.info("Using default URLs as fallback")

        # Return a default set of URLs as a fallback
        default_urls = [
            f"{settings.SOURCE_URL}",
            f"{settings.SOURCE_URL.rstrip('/')}/about",
            f"{settings.SOURCE_URL.rstrip('/')}/contact",
            f"{settings.SOURCE_URL.rstrip('/')}/documentation",
        ]

        logger.info(f"Using {len(default_urls)} default URLs as fallback")
        return default_urls

def extract_text_from_url_with_retry(url: str, max_retries: int = 3) -> str:
    """
    Extract text content from a given URL with retry logic.

    Args:
        url: The URL to extract text from
        max_retries: Maximum number of retry attempts

    Returns:
        Extracted text content
    """
    last_exception = None

    for attempt in range(max_retries):
        try:
            response = requests.get(url, timeout=30)  # 30 second timeout
            response.raise_for_status()

            # Parse HTML content and clean it
            soup = BeautifulSoup(response.content, 'html.parser')
            text = clean_html_content(soup)

            # Validate document size to ensure it doesn't exceed embedding token limits
            if len(text) > 100000:  # Roughly equivalent to token limits
                logger.warning(f"Document from {url} is very large ({len(text)} chars), may exceed embedding limits")
                # Optionally truncate or handle differently
                text = text[:100000]  # Truncate to max size

            logger.info(f"Successfully extracted text from {url}, length: {len(text)} characters")
            return text

        except requests.exceptions.RequestException as e:
            last_exception = e
            logger.warning(f"Attempt {attempt + 1} failed to fetch {url}: {e}")
            if attempt < max_retries - 1:
                time.sleep(2 ** attempt)  # Exponential backoff
            continue
        except Exception as e:
            logger.error(f"Error extracting text from {url}: {e}")
            raise

    # If all retries failed, raise the last exception
    logger.error(f"Failed to fetch {url} after {max_retries} attempts")
    raise last_exception

def extract_text_from_url(url: str) -> str:
    """
    Extract text content from a given URL.

    Args:
        url: The URL to extract text from

    Returns:
        Extracted text content
    """
    return extract_text_from_url_with_retry(url)

def clean_html_content(soup: BeautifulSoup) -> str:
    """
    Extract only main content from HTML using BeautifulSoup.

    Args:
        soup: BeautifulSoup object with HTML content

    Returns:
        Clean text content
    """
    # Look for main content containers first
    main_content = soup.find('main') or soup.find('article') or soup.find('div', class_=re.compile("content|main|post|article", re.I))

    if main_content:
        soup = main_content

    # Remove script and style elements
    for script in soup(["script", "style"]):
        script.decompose()

    # Remove navigation elements
    for element in soup(["nav", "header", "footer", "aside"]):
        element.decompose()

    # Remove elements with common class names for navigation and ads
    for element in soup.find_all(class_=re.compile("nav|menu|header|footer|sidebar|advertisement|ads|banner|cookie")):
        element.decompose()

    # Get text content
    text = soup.get_text()

    # Clean up text
    lines = (line.strip() for line in text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    text = ' '.join(chunk for chunk in chunks if chunk)

    return text

def handle_malformed_html_content(html_content: str) -> str:
    """
    Handle malformed HTML content.

    Args:
        html_content: Raw HTML content that may be malformed

    Returns:
        Clean text content extracted even from malformed HTML
    """
    try:
        # Try to parse with BeautifulSoup, which is quite tolerant to malformed HTML
        soup = BeautifulSoup(html_content, 'html.parser')
        return clean_html_content(soup)
    except Exception as e:
        logger.warning(f"Error parsing HTML with BeautifulSoup: {e}")
        # As a fallback, try to extract text with regex
        # Remove basic HTML tags but preserve content
        clean_text = re.sub(r'<[^>]+>', ' ', html_content)
        return clean_text

def chunk_text(text: str, chunk_size: int = None, chunk_overlap: int = None) -> List[str]:
    """
    Split text into appropriately sized segments.

    Args:
        text: The text to chunk
        chunk_size: Size of each chunk (default from config)
        chunk_overlap: Number of characters to overlap between chunks (default from config)

    Returns:
        List of text chunks
    """
    # Use config values if not provided
    if chunk_size is None:
        chunk_size = settings.CHUNK_SIZE
    if chunk_overlap is None:
        chunk_overlap = settings.CHUNK_OVERLAP

    # For a more sophisticated approach, we'll chunk by characters while preserving sentences
    sentences = re.split(r'[.!?]+', text)

    chunks = []
    current_chunk = ""

    for sentence in sentences:
        sentence = sentence.strip()
        if sentence:  # Only process non-empty sentences
            if len(current_chunk) + len(sentence) < chunk_size:
                current_chunk += " " + sentence
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())

                # Apply overlap by taking the last part of the previous chunk
                overlap_start = max(0, len(current_chunk) - chunk_overlap)
                current_chunk = current_chunk[overlap_start:] + " " + sentence

    # Add the last chunk if it's not empty
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    logger.info(f"Text chunked into {len(chunks)} chunks (size: {chunk_size}, overlap: {chunk_overlap})")
    return chunks

def embed(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings using Cohere API.

    Args:
        texts: List of text chunks to embed

    Returns:
        List of embedding vectors
    """
    client = initialize_cohere_client()

    try:
        response = client.embed(
            texts=texts,
            model=settings.COHERE_EMBED_MODEL,  # Using configurable Cohere embedding model
            input_type="search_document"  # Appropriate input type for document search
        )

        embeddings = [embedding for embedding in response.embeddings]
        logger.info(f"Generated embeddings for {len(texts)} text chunks using model {settings.COHERE_EMBED_MODEL}")
        return embeddings
    except Exception as e:
        logger.error(f"Failed to generate embeddings: {e}")
        raise

def create_collection(client: QdrantClient, collection_name: str = "chatbot_embedding"):
    """
    Create a collection in Qdrant for storing embeddings.

    Args:
        client: Initialized Qdrant client
        collection_name: Name of the collection to create
    """
    try:
        # Check if collection already exists
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if collection_name in collection_names:
            logger.info(f"Collection '{collection_name}' already exists")
            return

        # Create new collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
        )

        logger.info(f"Created collection '{collection_name}' in Qdrant")
    except Exception as e:
        logger.error(f"Failed to create collection '{collection_name}': {e}")
        raise

def compute_content_hash(text: str) -> str:
    """
    Compute a hash of the text content for duplicate detection.

    Args:
        text: The text to hash

    Returns:
        Hash of the text content
    """
    import hashlib
    return hashlib.md5(text.encode('utf-8')).hexdigest()

def save_chunk_to_qdrant(client: QdrantClient, text_chunk: str, embedding: List[float],
                         source_url: str, chunk_position: int, collection_name: str = "chatbot_embedding"):
    """
    Save a text chunk and its embedding to Qdrant with metadata.

    Args:
        client: Initialized Qdrant client
        text_chunk: The text chunk to store
        embedding: The embedding vector for the text chunk
        source_url: The URL where the content originated
        chunk_position: The position of this chunk in the original document
        collection_name: Name of the collection to store in
    """
    try:
        # Compute content hash to detect duplicates
        content_hash = compute_content_hash(text_chunk)

        # Create a unique ID for this chunk
        import uuid
        chunk_id = str(uuid.uuid4())

        # Store in Qdrant with content hash
        from datetime import datetime
        client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload={
                        "text": text_chunk,
                        "source_url": source_url,
                        "chunk_position": chunk_position,
                        "content_hash": content_hash,
                        "created_at": datetime.now().isoformat()
                    }
                )
            ]
        )

        logger.info(f"Saved chunk to Qdrant with ID: {chunk_id}")
    except Exception as e:
        logger.error(f"Failed to save chunk to Qdrant: {e}")
        raise

def check_duplicate_content(client: QdrantClient, text_chunk: str, collection_name: str = "chatbot_embedding") -> bool:
    """
    Check if the content already exists in Qdrant (duplicate detection).

    Args:
        client: Initialized Qdrant client
        text_chunk: The text chunk to check
        collection_name: Name of the collection to check in

    Returns:
        True if content is a duplicate, False otherwise
    """
    content_hash = compute_content_hash(text_chunk)

    try:
        # Search for points with the same content hash
        results = client.scroll(
            collection_name=collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="content_hash",
                        match=models.MatchValue(value=content_hash)
                    )
                ]
            ),
            limit=1
        )

        if results[0] if results else None:
            logger.info(f"Duplicate content detected with hash: {content_hash}")
            return True
        return False
    except Exception as e:
        # If search fails, log and return False to proceed with saving
        logger.warning(f"Failed to check for duplicate content: {e}")
        return False

def initialize_cohere_client():
    """Initialize and return a Cohere client using the API key from config."""
    try:
        client = cohere.Client(api_key=settings.COHERE_API_KEY)
        logger.info("Cohere client initialized successfully")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize Cohere client: {e}")
        raise

def initialize_qdrant_client():
    """Initialize and return a Qdrant client using connection details from config."""
    try:
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            port=settings.QDRANT_PORT
        )
        logger.info("Qdrant client initialized successfully")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant client: {e}")
        raise

def main():
    """Main function to orchestrate the ingestion pipeline."""
    logger.info("Starting RAG Content Ingestion Pipeline")

    # Initialize clients
    qdrant_client = initialize_qdrant_client()

    # Create the collection for storing embeddings
    create_collection(qdrant_client, "chatbot_embedding")

    # Get all URLs from the sitemap
    urls = get_all_urls()

    # Process each URL
    total_processed = 0
    total_chunks = 0
    for url in urls:  # Process all URLs from the sitemap
        try:
            logger.info(f"Processing URL: {url}")

            # Extract text from the URL
            text = extract_text_from_url(url)

            # Chunk the text
            chunks = chunk_text(text, settings.CHUNK_SIZE)

            # Embed the chunks
            embeddings = embed(chunks)

            # Save each chunk to Qdrant with metadata
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                # Check for duplicate content before saving
                if check_duplicate_content(qdrant_client, chunk, "chatbot_embedding"):
                    logger.info(f"Skipping duplicate content in {url}, chunk {i}")
                    continue

                save_chunk_to_qdrant(
                    client=qdrant_client,
                    text_chunk=chunk,
                    embedding=embedding,
                    source_url=url,
                    chunk_position=i
                )
                total_chunks += 1

            total_processed += 1
            logger.info(f"Successfully processed {url} ({len(chunks)} chunks, {total_chunks} total saved)")

        except Exception as e:
            logger.error(f"Error processing URL {url}: {e}")
            # Continue with next URL instead of stopping the entire process

    logger.info(f"RAG Content Ingestion Pipeline completed. Processed {total_processed}/{len(urls)} URLs, saved {total_chunks} chunks")


if __name__ == "__main__":
    main()