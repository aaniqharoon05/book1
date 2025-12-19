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
import xml.etree.ElementTree as ET
from urllib.parse import urljoin, urlparse
import hashlib
import uuid
from datetime import datetime
import os
from dotenv import load_dotenv

# Load environment variables from .env file
# Try to load from the same directory as this file
script_dir = os.path.dirname(os.path.abspath(__file__))
dotenv_path = os.path.join(script_dir, '.env')
load_dotenv(dotenv_path)

# Define settings directly from environment variables
class Settings:
    def __init__(self):
        # Qdrant Configuration
        self.QDRANT_URL: str = os.getenv("QDRANT_URL", "")
        self.QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
        self.QDRANT_PORT: int = int(os.getenv("QDRANT_PORT", "6333"))
        self.QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "chatbot_embedding")

        # Cohere Configuration
        self.COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
        self.COHERE_EMBED_MODEL: str = os.getenv("COHERE_EMBED_MODEL", "embed-multilingual-v3.0")

        # Source Configuration
        self.SOURCE_URL: str = os.getenv("SOURCE_URL", "")
        self.SITEMAP_URL: str = os.getenv("SITEMAP_URL", "")

        # Content Processing Configuration
        self.CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", "512"))
        self.CHUNK_OVERLAP: int = int(os.getenv("CHUNK_OVERLAP", "50"))

        # Logging Configuration
        self.LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

settings = Settings()

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
    Extract only main content from HTML using BeautifulSoup, focusing specifically on book content.

    Args:
        soup: BeautifulSoup object with HTML content

    Returns:
        Clean text content
    """
    # Define selectors for book content elements in common documentation/book platforms
    content_selectors = [
        # Docusaurus-specific content containers
        'main div.container article',
        'main div.theme-doc-markdown',
        'article.main-article',
        'div.doc-content',
        'div.markdown',

        # Generic content containers
        'main',
        'article',
        'div.content',
        'div.main-content',
        'div.post-content',
        'div.article-content',

        # Book-specific selectors
        'div.chapter-content',
        'section.book-content',
        'div.page-content',
        'main#content',
        'div#main-content',

        # Common class names for book/content areas
        'div[class*="content"]',
        'div[class*="article"]',
        'div[class*="post"]',
        'div[class*="chapter"]',
        'div[class*="page"]'
    ]

    # Try to find main content area with these selectors
    main_content = None

    # More specific search for book documentation platforms
    for selector in content_selectors:
        main_content = soup.select_one(selector)
        if main_content:
            break

    # If no specific content found, try looking for elements that usually hold book content
    if not main_content:
        # Look for content within specific tag patterns
        content_containers = soup.find_all(['main', 'article', 'section'])
        for container in content_containers:
            # Check if the container has attributes that suggest it's content-focused
            role_attr = container.get('role', '')
            aria_labelledby = container.get('aria-labelledby', '')

            if 'main' in role_attr or 'document' in aria_labelledby or len(container.get_text(strip=True)) > 100:
                main_content = container
                break

    # If still no content found, use the body tag
    if not main_content:
        main_content = soup.body or soup.html

    # Clone the content to work with
    content_soup = BeautifulSoup(str(main_content), 'html.parser')

    # More comprehensive removal of non-book content elements
    non_content_selectors = [
        # Navigation and menus
        'nav',
        'header',
        'footer',
        'aside',
        '.nav',
        '.navbar',
        '.menu',
        '.sidebar',
        '.toc',  # Table of Contents
        '.table-of-contents',

        # Common non-content classes
        '.header',
        '.footer',
        '.breadcrumb',
        '.pagination',
        '.tag',
        '.tags',
        '.author',
        '.authors',
        '.social-share',
        '.share-buttons',
        '.advertisement',
        '.ads',
        '.banner',
        '.cookie-consent',
        '.consent-banner',
        '.newsletter-signup',
        '.signup-form',
        '.related-posts',
        '.comments-section',
        '.disqus_thread',
        '.advertisement',
        '.promo',
        '.callout',
        '.alert',
        '.notification',
        '.top-link',
        '.prev-next-links',
        '.next-prev-navigation',

        # Common IDs for non-content areas
        '#nav',
        '#navbar',
        '#menu',
        '#sidebar',
        '#toc',
        '#footer',
        '#header',
        '#breadcrumb',
        '#pagination'
    ]

    for selector in non_content_selectors:
        for element in content_soup.select(selector):
            element.decompose()

    # Remove elements with specific class names related to navigation/pagination
    for element in content_soup.find_all(class_=re.compile(
        r'(nav|menu|header|footer|sidebar|advertisement|ads|banner|cookie|breadcrumb|'
        r'pagination|tag|author|social|share|related|comment|disqus|promo|callout|'
        r'alert|notification|top-link|\bnext\b|\bprev\b|\bpagination\b|toc|table.*?contents)'
    )):
        element.decompose()

    # Remove script and style elements
    for element in content_soup(["script", "style", "meta", "link"]):
        element.decompose()

    # Focus on elements that typically contain actual book content
    content_elements = content_soup.find_all([
        'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
        'p', 'div', 'span', 'pre', 'code', 'blockquote',
        'ol', 'ul', 'li', 'dl', 'dt', 'dd',
        'figure', 'figcaption', 'img', 'table', 'tr', 'td', 'th'
    ])

    # Filter content elements to include only those that don't match anti-patterns
    valid_content = []
    for elem in content_elements:
        # Skip elements that are likely non-content
        class_attr = elem.get('class', [])
        id_attr = elem.get('id', '')

        # Convert to lowercase for comparison
        class_list = ' '.join(class_attr).lower()

        # Skip if element contains navigation-like text content
        text_content = elem.get_text(strip=True)[:100].lower()
        nav_indicators = ['next', 'previous', 'nav', 'menu', 'table of contents', 'toc',
                         'chapter', 'part', 'section', 'index', 'outline', 'summary']

        is_navigation_like = any(indicator in text_content for indicator in nav_indicators)
        is_small_and_common = len(text_content) < 10 and any(word in text_content for word in
                              ['next', 'prev', 'home', 'top', 'up', 'back', 'more'])

        if not is_navigation_like and not is_small_and_common:
            valid_content.append(elem)

    # Extract clean text from valid content elements
    clean_texts = []
    for elem in valid_content:
        text = elem.get_text(separator=' ', strip=True)
        if text and len(text) > 5:  # Avoid very short text snippets
            clean_texts.append(text)

    # Join all valid content texts
    text = ' '.join(clean_texts)

    # Clean up text formatting
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

import re
from typing import List

def chunk_text(text: str, chunk_size: int = None, chunk_overlap: int = None) -> List[str]:
    """
    Split text into word-based chunks with overlap, preserving sentence boundaries.

    Args:
        text: The text to chunk
        chunk_size: Number of WORDS per chunk (default from settings)
        chunk_overlap: Number of WORDS to overlap between chunks (default from settings)

    Returns:
        List of clean text chunks
    """
    if chunk_size is None:
        chunk_size = settings.CHUNK_SIZE
    if chunk_overlap is None:
        chunk_overlap = settings.CHUNK_OVERLAP

    # Normalize whitespace
    text = re.sub(r'\s+', ' ', text).strip()

    # Split into sentences (keep punctuation)
    sentences = re.split(r'(?<=[.!?])\s+', text)

    chunks = []
    current_words = []

    for sentence in sentences:
        sentence_words = sentence.split()

        # If adding this sentence stays within chunk size
        if len(current_words) + len(sentence_words) <= chunk_size:
            current_words.extend(sentence_words)
        else:
            # Save current chunk
            if current_words:
                chunks.append(" ".join(current_words))

            # Start new chunk with overlap
            overlap_words = current_words[-chunk_overlap:] if chunk_overlap > 0 else []
            current_words = overlap_words + sentence_words

    # Add final chunk
    if current_words:
        chunks.append(" ".join(current_words))

    logger.info(
        f"Text chunked into {len(chunks)} chunks "
        f"(words per chunk: {chunk_size}, overlap: {chunk_overlap})"
    )

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
        logger.info(f"Attempting to connect to Qdrant at: {settings.QDRANT_URL}")
        logger.info(f"Using API key (first 10 chars): {settings.QDRANT_API_KEY[:10] if settings.QDRANT_API_KEY else 'None'}")
        logger.info(f"Using port: {settings.QDRANT_PORT}")

        # Check if this is a cloud instance URL
        if "cloud.qdrant.io" in settings.QDRANT_URL:
            logger.info("Detected Qdrant Cloud instance, initializing without port")
            # For cloud instances, don't specify port to avoid conflicts
            client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY
            )
        else:
            logger.info("Using local Qdrant instance")
            # For local instances, specify both URL and port
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


# Import required modules for parse_sitemap function
import xml.etree.ElementTree as ET
from typing import List
from urllib.parse import urljoin, urlparse


def parse_sitemap(sitemap_url: str) -> List[str]:
    """
    Parse a sitemap XML file and return a list of URLs.

    Args:
        sitemap_url: URL of the sitemap XML file

    Returns:
        List of URLs found in the sitemap
    """
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        # Parse the XML content
        root = ET.fromstring(response.content)

        # Handle namespaces - sitemap XML usually has a namespace
        namespace = {'sm': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

        urls = []
        for url_elem in root.findall('sm:url', namespace):
            loc_elem = url_elem.find('sm:loc', namespace)
            if loc_elem is not None and loc_elem.text:
                url = loc_elem.text.strip()
                # Exclude URLs containing "blog" in the path
                if '/blog' not in url and not url.endswith('/blog'):
                    urls.append(url)

        logger.info(f"Found {len(urls)} URLs in sitemap (excluding blog sections)")
        return urls

    except requests.RequestException as e:
        logger.error(f"Error fetching sitemap: {e}")
        raise
    except ET.ParseError as e:
        logger.error(f"Error parsing sitemap XML: {e}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error parsing sitemap: {e}")
        raise


def check_sitemap():
    """
    Function to test/check the sitemap by fetching and printing its content.
    This consolidates the functionality from check_sitemap.py
    """
    # Using the sitemap URL from settings, or a default if not available
    sitemap_url = getattr(settings, 'SITEMAP_URL', 'https://physical-ai-robotics-sable.vercel.app/sitemap.xml')

    response = requests.get(sitemap_url)
    print("Status code:", response.status_code)
    print("Response content (first 1000 chars):")
    print(response.text[:1000])


if __name__ == "__main__":
    main()