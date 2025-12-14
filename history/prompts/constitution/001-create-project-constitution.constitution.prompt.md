---
id: 001
title: Create Project Constitution
stage: constitution
date: 2025-12-08
surface: agent
model: gemini-pro
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
  null
---

## Prompt

/sp.constitution

Project: Book — “Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Robots”

Core Purpose:
Create a complete, industry-aligned, technically accurate book that teaches students how to design, simulate, and deploy humanoid robots using ROS 2, Gazebo, Unity, and NVIDIA Isaac. The book will serve as the official curriculum for the Physical AI & Humanoid Robotics capstone quarter.

Core Principles:
- Technical accuracy (robotics, AI, physics, ROS 2)
- Clarity for students with programming background (Python, AI)
- Progressive learning (simple → complex → integrated)
- Real-world applicability (simulation + hardware)
- Reproducibility (step-by-step, command-level clarity)
- Zero hallucination (all technical claims verifiable)
- Modular teaching (4 modules, each self-contained)

Writing Standards:
- Tone: Clear, educational, instructor-like
- Complexity: Intermediate → Advanced (gradually increasing)
- Use diagrams, command examples, architecture tables where needed
- Use correct robotics terminology
- Avoid ambiguous wording and generic AI explanations
- Use short paragraphs and bullets for readability

Source & Fact Requirements:
- All claims about ROS, Gazebo, Unity, and NVIDIA Isaac must reference official documentation.
- Hardware specifications must match vendor docs (NVIDIA, Intel RealSense, Unitree).
- No fictional features or invented API calls.
- All software instructions must be testable on Ubuntu 22.04.

Book Structure Standards:
- Book organized strictly around 4 official modules:
  1. The Robotic Nervous System (ROS 2)
  2. The Digital Twin (Gaze.specify/memory/constitution.mdbo & Unity)
  3. The AI-Robot Brain (NVIDIA Isaac)
  4. Vision-Language-Action (VLA)
- Each module explains: concepts → tools → hands-on → mini-project.
- Chapters must map to the weekly breakdown (Weeks 1–13).

Constraints:
- All diagrams must be reproducible (ASCII or simple SVG).
- All code examples must be executable.
- All robot-related procedures (URDF, sensor simulation, Nav2, VSLAM) must follow official configurations.
- Hardware descriptions must match realistic lab environments.
- No unnecessary academic theory; focus on applied Physical AI.

Success Criteria:
- Book fully deployable on Docusaurus & GitHub Pages.
- Students can complete all module mini-projects successfully.
- Robot simulation pipelines work end-to-end:
  - ROS 2 nodes → Gazebo → Isaac Sim → VLA planner
- Capstone project instructions complete:
  - Voice command → VLA plan → movement → object detection → manipulation.
- Content mirrors industry standards for humanoid robotics labs.

Non-Goals (Explicitly Out of Scope):
- Designing custom hardware from scratch.
- Deep robotics mathematics (control theory, advanced kinematics proofs).
- Reinforcement learning algorithms implementation from scratch.
- Research-level humanoid gait optimization.

Glossary Requirements:
- A complete glossary must be included (ROS terms, Isaac Sim terms, sensors, kinematics).
- Robotics acronyms must be defined on first use.

Output Format:
- Docusaurus-based book
- GitHub Pages deployment ready
- Includes diagrams, tables, code examples, and architecture views
- Each chapter includes:
  - Learning outcomes
  - Summary
  - Hands-on steps
  - Common errors & debugging section

Overall Commitment:
This constitution governs *all future specifications*, chapter drafts, and agent actions.  
All generated content must follow these principles without exception.

## Response snapshot

I have successfully updated the project constitution at `.specify/memory/constitution.md` based on your input. The new version is 1.0.0. I have also verified that the project templates are consistent with the new constitution.

## Outcome

- ✅ Impact: High - The constitution is the foundation for the project.
- 🧪 Tests: N/A
- 📁 Files: 1 created/updated
- 🔁 Next prompts: N/A
- 🧠 Reflection: The process of updating the constitution was smooth. The user provided clear and detailed information.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
