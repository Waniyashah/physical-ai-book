---
title: RAG-Powered Search
sidebar_position: 1
---

# RAG-Powered Search

Our documentation features an AI-powered search that leverages Retrieval-Augmented Generation (RAG) to provide contextual answers to your questions about Physical AI and Humanoid Robotics.

## How It Works

The RAG system combines:
- **Retrieval**: Searches through our documentation corpus to find relevant information
- **Generation**: Uses AI to synthesize answers based on the retrieved information
- **Grounding**: Provides source citations so you can verify the information

## Using the Search

Simply type your question in the search box and our system will:
1. Retrieve relevant sections from the documentation
2. Generate a contextual answer
3. Provide source links for verification

## Features

- Contextual answers tailored to your specific question
- Source citations for fact-checking
- Real-time processing
- Grounded responses based on our documentation

import RAGSearch from '@site/src/components/Search/RAGSearch';

<RAGSearch />

## Technical Details

The search functionality connects to our backend RAG service which:
- Integrates with Qdrant for vector storage
- Uses Cohere for embeddings
- Leverages AI models for response generation
- Ensures all responses are grounded in the source material