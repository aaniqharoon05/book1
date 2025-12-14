import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    """Configuration settings for the ingestion pipeline."""

    # Cohere settings
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    COHERE_EMBED_MODEL: str = os.getenv("COHERE_EMBED_MODEL", "embed-english-v3.0")

    # Qdrant settings
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_PORT: int = int(os.getenv("QDRANT_PORT", 6333))

    # Content processing settings
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", 512))
    CHUNK_OVERLAP: int = int(os.getenv("CHUNK_OVERLAP", 20))  # Number of characters to overlap
    SOURCE_URL: str = os.getenv("SOURCE_URL", "https://book1-eight.vercel.app/")
    SITEMAP_URL: str = f"{SOURCE_URL.rstrip('/')}/sitemap.xml"

    # Validation
    def validate(self):
        """Validate that required settings are present."""
        errors = []
        if not self.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is required")
        if not self.QDRANT_URL:
            errors.append("QDRANT_URL is required")

        if errors:
            raise ValueError(f"Configuration errors: {'; '.join(errors)}")

# Global settings instance
settings = Settings()

# Validate settings on import
settings.validate()