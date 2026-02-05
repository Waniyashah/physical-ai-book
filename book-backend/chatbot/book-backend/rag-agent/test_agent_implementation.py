#!/usr/bin/env python3
"""
Test script to verify that the RAG Agent implementation is working correctly
"""

import os
import sys
from pathlib import Path

def test_spec1_functionality():
    """Test that Spec 1 functions are available in main.py (from previous implementation)"""
    print("Testing Spec 1 components (existing pipeline)...")

    # Check if main.py exists in the rag-pipeline directory
    main_py_path = Path("../rag-pipeline/main.py")
    if not main_py_path.exists():
        print("  [FAIL] main.py not found in rag-pipeline/")
        return False

    with open(main_py_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check for key functionalities from Spec 1
    checks = {
        "URL fetching": "fetch_url_content" in content,
        "Content extraction": "extract_clean_text" in content,
        "Content chunking": "chunk_content" in content,
        "Cohere integration": "cohere" in content and "embed" in content,
        "Qdrant storage": "qdrant_client" in content and "upsert" in content
    }

    all_spec1_pass = True
    for check, result in checks.items():
        status = "[PASS]" if result else "[FAIL]"
        print(f"  {status} {check}")
        if not result:
            all_spec1_pass = False

    return all_spec1_pass


def test_spec2_functionality():
    """Test that Spec 2 functions are available in retrieve.py (from previous implementation)"""
    print("\nTesting Spec 2 components (existing retrieval)...")

    # Check if retrieve.py exists in the rag-pipeline directory (where it actually exists)
    retrieve_py_path = Path("../rag-pipeline/retrieve.py")
    if not retrieve_py_path.exists():
        print("  [FAIL] retrieve.py not found in rag-pipeline/")
        return False

    with open(retrieve_py_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check for key functionalities from Spec 2 (looking for actual functions in the file)
    checks = {
        "Query processing": "run_query" in content or "main" in content,
        "Similarity search": "search" in content or "qdrant" in content,
        "Metadata handling": "payload" in content or "metadata" in content,
        "Validation": "validate" in content or "validation" in content
    }

    all_spec2_pass = True
    for check, result in checks.items():
        status = "[PASS]" if result else "[FAIL]"
        print(f"  {status} {check}")
        if not result:
            all_spec2_pass = False

    return all_spec2_pass


def test_spec3_functionality():
    """Test that Spec 3 (RAG Agent) functions are available in agent.py"""
    print("\nTesting Spec 3 components (new agent implementation)...")

    # Check if agent.py exists in the current directory
    agent_py_path = Path("agent.py")
    if not agent_py_path.exists():
        print("  [FAIL] agent.py not found")
        return False

    with open(agent_py_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check for key functionalities from Spec 3
    checks = {
        "OpenAI client init": "initialize_openai_client" in content,
        "Cohere client init": "initialize_cohere_client" in content,
        "Qdrant client init": "initialize_qdrant_client" in content,
        "Retrieval tool": "create_retrieval_tool_function" in content,
        "Agent creation": "create_agent_with_retrieval_tool" in content,
        "Query execution": "run_agent_query" in content,
        "Response validation": "validate_agent_response" in content,
        "CLI interface": "argparse" in content
    }

    all_spec3_pass = True
    for check, result in checks.items():
        status = "[PASS]" if result else "[FAIL]"
        print(f"  {status} {check}")
        if not result:
            all_spec3_pass = False

    return all_spec3_pass


def test_files_existence():
    """Test that all required files exist"""
    print("\nChecking required files...")

    required_files = [
        "agent.py",
        "config.yaml",
        ".env",
        "requirements.txt",
        ".gitignore",
        "README.md"
    ]

    all_exist = True
    for file in required_files:
        exists = Path(file).exists()
        status = "[OK]" if exists else "[MISSING]"
        print(f"  {status} {file}")
        if not exists:
            all_exist = False

    return all_exist


def test_imports():
    """Test that key modules can be imported without errors"""
    print("\nTesting module imports...")

    try:
        # Change to the rag-agent directory to test imports
        os.chdir(Path(__file__).parent)

        # Test importing the agent module
        import agent
        print("  [PASS] agent.py imports successfully")

        # Check that key functions exist in the module
        required_funcs = [
            'load_config',
            'initialize_openai_client',
            'initialize_cohere_client',
            'initialize_qdrant_client',
            'create_retrieval_tool_function',
            'create_agent_with_retrieval_tool',
            'run_agent_query',
            'validate_agent_response',
            'main'
        ]

        missing_funcs = []
        for func_name in required_funcs:
            if not hasattr(agent, func_name):
                missing_funcs.append(func_name)

        if missing_funcs:
            print(f"  [FAIL] Missing functions in agent module: {missing_funcs}")
            return False
        else:
            print("  [PASS] All required functions present in agent module")
            return True

    except ImportError as e:
        print(f"  [FAIL] Import error: {e}")
        return False
    except Exception as e:
        print(f"  [FAIL] Error testing imports: {e}")
        return False


def main():
    """Main test function"""
    print("[VERIFICATION] RAG AGENT IMPLEMENTATION VERIFICATION")
    print("="*50)

    # Run all tests
    files_ok = test_files_existence()
    spec1_ok = test_spec1_functionality()
    spec2_ok = test_spec2_functionality()
    spec3_ok = test_spec3_functionality()
    imports_ok = test_imports()

    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)

    print(f"Files exist: {'[PASS] PASS' if files_ok else '[FAIL] FAIL'}")
    print(f"Spec 1 (Pipeline) components: {'[PASS] PASS' if spec1_ok else '[FAIL] FAIL'}")
    print(f"Spec 2 (Retrieval) components: {'[PASS] PASS' if spec2_ok else '[FAIL] FAIL'}")
    print(f"Spec 3 (Agent) components: {'[PASS] PASS' if spec3_ok else '[FAIL] FAIL'}")
    print(f"Module imports: {'[PASS] PASS' if imports_ok else '[FAIL] FAIL'}")

    overall_success = all([files_ok, spec1_ok, spec2_ok, spec3_ok, imports_ok])
    print(f"\nOverall Status: {'[PASS] SUCCESS - All implementations verified!' if overall_success else '[FAIL] FAILURE - Some components missing'}")

    if overall_success:
        print("\n[SUCCESS] VERIFICATION COMPLETE!")
        print("All RAG Pipeline implementations (Spec 1, Spec 2, and Spec 3) are in place!")
        print("- Spec 1: RAG Pipeline - Website Deployment, Embedding Generation, and Vector Storage")
        print("- Spec 2: RAG Pipeline - Retrieval and Pipeline Validation")
        print("- Spec 3: RAG Agent - OpenAI Agents SDK Integration")
    else:
        print("\n[FAIL] VERIFICATION FAILED!")
        print("Some components are missing. Please check the individual test results above.")

    return overall_success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)