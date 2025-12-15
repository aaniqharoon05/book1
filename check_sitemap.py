import requests

# Fetch the sitemap
response = requests.get('https://physical-ai-kappa.vercel.app/sitemap.xml')
print("Status code:", response.status_code)
print("Response content (first 1000 chars):")
print(response.text[:1000])