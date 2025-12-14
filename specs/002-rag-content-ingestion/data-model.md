# Data Model: RAG Content Ingestion and Vectorization

## Overview
This document defines the data structures and entities for the RAG content ingestion pipeline that converts book content into searchable vector embeddings.

## Key Entities

### 1. BookContent
Represents the source material to be ingested, including URLs, titles, and version information.

**Fields:**
- `url` (string): The URL of the book content to be ingested
- `title` (string): The title of the book page/section
- `version` (string): Version information of the content
- `last_modified` (datetime): Timestamp of when the content was last modified
- `content_hash` (string): Hash of the content to detect changes

**Validation:**
- URL must be a valid HTTP/HTTPS URL
- Title must not be empty
- Content hash is required for change detection

### 2. TextChunk
Represents a segment of cleaned and processed content, including position metadata and embedding vector.

**Fields:**
- `id` (string): Unique identifier for the chunk
- `content` (string): The actual text content of the chunk
- `position` (number): The position of this chunk in the original document
- `source_url` (string): The URL from which this chunk was extracted
- `source_title` (string): The title of the source document
- `metadata` (object): Additional metadata about the chunk

**Validation:**
- Content must be non-empty
- Position must be a non-negative integer
- Source URL must be valid
- Maximum content length should be configurable (default 512 tokens)

### 3. EmbeddingVector
Numeric representation of text content suitable for similarity comparison in vector space.

**Fields:**
- `chunk_id` (string): Reference to the TextChunk this embedding represents
- `vector` (array of numbers): The actual embedding vector from Cohere
- `model_name` (string): Name of the model used to generate the embedding
- `created_at` (datetime): When this embedding was created

**Validation:**
- Vector must have consistent dimensions based on the embedding model
- Chunk_id must reference a valid TextChunk
- Model name is required

### 4. Metadata
Information associated with each text chunk, including source URL, position within document, and timestamps.

**Fields:**
- `source_url` (string): URL where the content originated
- `chunk_position` (number): Position of this chunk in the original document
- `document_title` (string): Title of the source document
- `page_number` (number): Page number in the original document (if applicable)
- `section_title` (string): Section title of the content
- `created_at` (datetime): Timestamp when metadata was created
- `updated_at` (datetime): Timestamp when metadata was last updated

### 5. QdrantCollection
Container for storing embeddings and metadata with vector search capabilities.

**Fields:**
- `name` (string): Name of the collection ("chatbot_embedding")
- `vector_size` (number): Dimension of the vectors stored in this collection
- `distance_function` (string): Distance function used for similarity search ("Cosine", "Euclidean", etc.)
- `count` (number): Number of vectors in the collection

**Validation:**
- Name must be unique within Qdrant instance
- Vector size must match embedding model output
- Distance function must be supported by Qdrant

## Relationships

### BookContent → TextChunk
- One BookContent can generate multiple TextChunks
- Relationship: One-to-Many
- Cardinality: 1 to many

### TextChunk → EmbeddingVector
- One TextChunk has exactly one EmbeddingVector
- Relationship: One-to-One
- Cardinality: 1 to 1

### TextChunk → Metadata
- One TextChunk has exactly one Metadata object
- Relationship: One-to-One
- Cardinality: 1 to 1

## State Transitions

### TextChunk States
1. **PENDING** - Content extracted but not yet processed
2. **CHUNKED** - Content has been split into appropriate chunks
3. **EMBEDDED** - Embeddings have been generated for this chunk
4. **STORED** - Embedding has been saved to Qdrant

## Schema Implementation

```python
# Python data classes that represent these entities

from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Dict, Any

@dataclass
class BookContent:
    url: str
    title: str
    version: str = ""
    last_modified: Optional[datetime] = None
    content_hash: str = ""

@dataclass
class TextChunk:
    id: str
    content: str
    position: int
    source_url: str
    source_title: str
    metadata: Dict[str, Any]

@dataclass
class EmbeddingVector:
    chunk_id: str
    vector: List[float]
    model_name: str
    created_at: datetime

@dataclass
class Metadata:
    source_url: str
    chunk_position: int
    document_title: str
    page_number: Optional[int] = None
    section_title: Optional[str] = None
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None

@dataclass
class QdrantCollection:
    name: str
    vector_size: int
    distance_function: str
    count: int = 0
```

## Validation Rules

1. **URL Validation**: All URLs must pass validation through a standard URL validator
2. **Content Quality**: Text chunks must meet minimum length requirements to ensure semantic meaning
3. **Embedding Consistency**: All embeddings in a collection must have the same dimensionality
4. **Hash Verification**: Content hashes are used to prevent redundant processing of unchanged content
5. **Position Continuity**: Chunk positions within a document must form a continuous sequence