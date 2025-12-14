import xml.etree.ElementTree as ET
from typing import List
import requests
from urllib.parse import urljoin, urlparse
import logging

logger = logging.getLogger(__name__)

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
                urls.append(loc_elem.text.strip())
        
        logger.info(f"Found {len(urls)} URLs in sitemap")
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