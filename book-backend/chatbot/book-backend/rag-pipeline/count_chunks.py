#!/usr/bin/env python3
"""Script to count total chunks stored in Qdrant during pipeline run"""

import subprocess
import re

def count_chunks_from_pipeline():
    print("Running pipeline and counting stored chunks...")

    # Run the pipeline and capture output
    result = subprocess.run(['python', 'main.py'],
                           capture_output=True, text=True,
                           cwd=r'C:\Users\Wahab\OneDrive\Desktop\hackathon-book\book-backend\chatbot\book-backend\rag-pipeline')

    # Extract all "Stored X chunks" messages
    output = result.stdout + result.stderr
    chunk_matches = re.findall(r'Stored (\d+) chunks', output)

    if chunk_matches:
        chunk_counts = [int(match) for match in chunk_matches]
        total_chunks = sum(chunk_counts)

        print(f"Individual chunk counts per URL: {chunk_counts}")
        print(f"Total URLs processed: {len(chunk_counts)}")
        print(f"Total chunks stored: {total_chunks}")

        return total_chunks
    else:
        print("No 'Stored X chunks' messages found in output")
        return 0

if __name__ == "__main__":
    count_chunks_from_pipeline()