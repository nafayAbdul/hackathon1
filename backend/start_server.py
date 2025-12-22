#!/usr/bin/env python3
"""
RAG Chatbot Startup Script
Ensures all required services and configurations are in place before starting the server
"""
import os
import sys
import subprocess
import time
from pathlib import Path

def check_environment_variables():
    """Check if required environment variables are set"""
    required_vars = [
        'GOOGLE_API_KEY',
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'NEON_DB_URL'
    ]

    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print(f"[ERROR] Missing required environment variables: {', '.join(missing_vars)}")
        print("\n[INFO] Please set these variables in your .env file or environment:")
        print("[INFO] - GOOGLE_API_KEY: Your Google API key for Gemini")
        print("[INFO] - QDRANT_URL: URL for your Qdrant instance")
        print("[INFO] - QDRANT_API_KEY: API key for Qdrant")
        print("[INFO] - NEON_DB_URL: Connection string for Neon PostgreSQL database")
        return False

    print("[SUCCESS] All required environment variables are set")
    return True

def check_dependencies():
    """Check if required Python packages are installed"""
    required_packages = [
        'fastapi',
        'uvicorn',
        'python_dotenv',
        'requests',
        'beautifulsoup4',
        'qdrant_client',
        'pydantic',
        'langchain',
        'langchain_community',
        'langchain_google_genai',
        'sqlalchemy',
        'psycopg2_binary'
    ]

    missing_packages = []
    for package in required_packages:
        try:
            __import__(package.replace('-', '_'))
        except ImportError:
            missing_packages.append(package.replace('_', '-'))

    if missing_packages:
        print(f"[ERROR] Missing required Python packages: {', '.join(missing_packages)}")
        return False

    print("[SUCCESS] All required Python packages are installed")
    return True

def start_server():
    """Start the FastAPI server"""
    print("🚀 Starting RAG Chatbot API server...")
    
    # Change to backend directory
    backend_dir = Path(__file__).parent
    os.chdir(backend_dir)
    
    # Start the server using uvicorn
    cmd = [sys.executable, "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
    
    try:
        process = subprocess.Popen(cmd)
        print("✅ RAG Chatbot API server started successfully!")
        print("🌐 Server running at: http://0.0.0.0:8000")
        print("💡 Health check: http://0.0.0.0:8000/health")
        print("🔄 Press Ctrl+C to stop the server")
        
        # Wait for the process to complete
        process.wait()
    except KeyboardInterrupt:
        print("\n🛑 Shutting down server...")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
        print("✅ Server stopped")
    except Exception as e:
        print(f"❌ Error starting server: {e}")
        sys.exit(1)

def main():
    print("[ROBOT] RAG Chatbot Startup Sequence Initiated")
    print("="*50)

    # Load environment variables first
    from dotenv import load_dotenv
    load_dotenv()

    # Check environment variables
    if not check_environment_variables():
        sys.exit(1)
    
    # Check dependencies
    if not check_dependencies():
        sys.exit(1)
    
    # Everything is set up, start the server
    start_server()

if __name__ == "__main__":
    main()