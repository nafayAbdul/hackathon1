#!/usr/bin/env python3
"""
Test script to verify API key connectivity and service health
"""
import os
import requests
from pathlib import Path
from dotenv import load_dotenv

def test_google_api():
    """Test Google API key connectivity"""
    print("[TEST] Testing Google API connectivity...")

    # Load environment variables
    load_dotenv()

    api_key = os.getenv('GOOGLE_API_KEY')
    if not api_key:
        print("[ERROR] GOOGLE_API_KEY environment variable not set")
        return False

    try:
        # Test the API with a simple models request
        url = f"https://generativelanguage.googleapis.com/v1beta/models?key={api_key}"
        response = requests.get(url, timeout=10)

        if response.status_code == 200:
            print("[SUCCESS] Google API key is valid and working")
            models = response.json()
            print(f"[INFO] Available models: {len(models.get('models', []))} models found")
            return True
        else:
            print(f"[ERROR] Google API returned status code {response.status_code}")
            print(f"[INFO] Response: {response.text}")
            return False
    except Exception as e:
        print(f"[ERROR] Error testing Google API: {e}")
        return False

def test_qdrant_connection():
    """Test Qdrant connection"""
    print("\n[TEST] Testing Qdrant connection...")

    # Load environment variables
    load_dotenv()

    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')

    if not qdrant_url:
        print("[ERROR] QDRANT_URL environment variable not set")
        return False

    if not qdrant_api_key:
        print("[ERROR] QDRANT_API_KEY environment variable not set")
        return False

    try:
        # Import Qdrant client
        from qdrant_client import QdrantClient

        # Create client instance
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )

        # Test connection by getting cluster info
        cluster_info = client.get_collections()
        print("[SUCCESS] Qdrant connection successful")
        print(f"[INFO] Collections: {len(cluster_info.collections)} collections found")
        return True
    except Exception as e:
        print(f"[ERROR] Error testing Qdrant connection: {e}")
        return False

def test_neon_db_connection():
    """Test Neon DB connection"""
    print("\n[TEST] Testing Neon DB connection...")

    # Load environment variables
    load_dotenv()

    neon_db_url = os.getenv('NEON_DB_URL')
    if not neon_db_url:
        print("[ERROR] NEON_DB_URL environment variable not set")
        return False

    try:
        import psycopg2

        # Test connection
        conn = psycopg2.connect(neon_db_url)
        cur = conn.cursor()

        # Execute a simple query
        cur.execute("SELECT version();")
        version = cur.fetchone()

        print("[SUCCESS] Neon DB connection successful")
        print(f"[INFO] Database version: {version[0][:50]}...")

        # Close connection
        cur.close()
        conn.close()
        return True
    except Exception as e:
        print(f"[ERROR] Error testing Neon DB connection: {e}")
        return False

def main():
    print("[TEST] Service Connectivity Test Suite")
    print("="*50)

    # Change to backend directory
    backend_dir = Path(__file__).parent
    os.chdir(backend_dir)

    # Run all tests
    results = []
    results.append(("Google API", test_google_api()))
    results.append(("Qdrant", test_qdrant_connection()))
    results.append(("Neon DB", test_neon_db_connection()))

    print("\n[SUMMARY] Test Results Summary:")
    print("-" * 30)

    all_passed = True
    for service, result in results:
        status = "[PASS]" if result else "[FAIL]"
        print(f"{service:<15} {status}")
        if not result:
            all_passed = False

    print("-" * 30)
    if all_passed:
        print("[SUCCESS] All services are accessible and working correctly!")
        print("[INFO] You can now start the RAG Chatbot server with: python start_server.py")
    else:
        print("[WARNING] Some services failed connectivity tests.")
        print("[INFO] Please verify your environment variables and network connectivity.")

    return all_passed

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)