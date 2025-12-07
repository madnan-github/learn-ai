🚀 Project Overview
An AI-native interactive textbook for teaching Physical AI & Humanoid Robotics, developed as part of the Panaversity Hackathon I. This project combines comprehensive educational content with cutting-edge AI features to create a next-generation learning experience.

Live Demo: https://madnan-github.github.io/learn-ai/
Repository: https://github.com/madnan-github/learn-ai

🎯 Hackathon Objectives
This project addresses the Panaversity Hackathon I challenge to create a textbook for teaching Physical AI & Humanoid Robotics with the following deliverables:

Core Requirements (100 Points)
✅ AI/Spec-Driven Book Creation: Docusaurus-based textbook using Spec-Kit Plus and Claude Code

✅ Integrated RAG Chatbot: Retrieval-Augmented Generation system with OpenAI Agents, FastAPI, Neon Postgres, and Qdrant Cloud

✅ Text Selection Capability: Chatbot answers questions based on selected text from the book

Bonus Features (Up to 200 Extra Points)
✅ Reusable Intelligence: Claude Code Subagents and Agent Skills implementation

✅ Authentication System: Better-auth integration with user background profiling

✅ Content Personalization: Adaptive content based on user's software/hardware background

✅ Urdu Translation: One-click translation of content to Urdu language

📚 Course Structure
Quarter Overview
This capstone quarter introduces Physical AI—AI systems that function in reality and comprehend physical laws. Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions.

Four Core Modules
Module 1: The Robotic Nervous System (ROS 2)
ROS 2 Nodes, Topics, and Services

Bridging Python Agents to ROS controllers using rclpy

Understanding URDF for humanoids

Building ROS 2 packages with Python

Module 2: The Digital Twin (Gazebo & Unity)
Physics simulation in Gazebo

High-fidelity rendering in Unity

Sensor simulation: LiDAR, Depth Cameras, IMUs

URDF and SDF robot description formats

Module 3: The AI-Robot Brain (NVIDIA Isaac™)
NVIDIA Isaac Sim for photorealistic simulation

Isaac ROS for hardware-accelerated VSLAM

Nav2 for bipedal humanoid movement

Reinforcement learning for robot control

Module 4: Vision-Language-Action (VLA)
OpenAI Whisper for voice commands

LLMs for natural language task planning

Multi-modal interaction systems

Capstone Project: Autonomous Humanoid with voice control

🤖 Technical Features
1. AI-Powered RAG Chatbot
Backend: FastAPI with OpenAI Agents SDK

Vector Database: Qdrant Cloud Free Tier

Relational Database: Neon Serverless Postgres

Features: Context-aware responses, text selection capability, conversation history

2. Personalized Learning Experience
Authentication: Better-auth with custom profile fields

Adaptive Content: Content adjusts based on user's background (beginner, intermediate, expert)

Skill Assessment: Questions about software/hardware experience during signup

3. Multi-language Accessibility
Urdu Translation: One-click chapter translation

RTL Support: Proper right-to-left text rendering

Font Optimization: Noto Nastaliq Urdu font integration

4. Claude Code Integration
Spec-Kit Plus: For structured book development

Reusable Subagents: Technical writer, code review, content validation agents

Agent Skills: Custom skills for book-specific tasks

💻 Hardware Requirements
Option A: On-Premise Lab (Recommended)
Component	Specification	Cost
Workstation	NVIDIA RTX 4070 Ti (12GB+), 64GB RAM, Ubuntu 22.04	~$2,500
Edge Kit	NVIDIA Jetson Orin Nano, Intel RealSense D435i	~$700
Robot	Unitree Go2 Edu (quadruped proxy)	~$2,500
Option B: Cloud-Native Approach
Cloud Workstation: AWS g5.2xlarge (~$205/quarter)

Local Edge Kit: NVIDIA Jetson Orin Nano ($249)

Shared Robot: Unitree Go2 (~$3,000 one-time)

Budget-Friendly Option
Jetson Student Kit: ~$700 total

Simulation-First: Focus on Gazebo/Isaac Sim before physical hardware

Cloud Simulations: Use NVIDIA Omniverse Cloud for GPU-intensive tasks

🛠️ Technical Architecture
Frontend (Docusaurus)
text
/book
├── /docs
│   ├── /module-1-ros2
│   ├── /module-2-gazebo-unity
│   ├── /module-3-nvidia-isaac
│   └── /module-4-vla
├── /src/components
│   ├── ChatbotWidget.jsx
│   ├── PersonalizeButton.jsx
│   ├── TranslateButton.jsx
│   └── UserProfile.jsx
└── /static
Backend Services
text
/backend
├── /api
│   ├── auth.py (Better-auth integration)
│   ├── chatbot.py (RAG implementation)
│   ├── personalize.py (Content adaptation)
│   └── translate.py (Urdu translation)
├── /models
│   ├── embeddings.py (Qdrant integration)
│   └── database.py (Neon Postgres models)
└── main.py (FastAPI app)
Database Schema
sql
-- Neon Postgres Tables
CREATE TABLE users (
    id UUID PRIMARY KEY,
    email VARCHAR(255) UNIQUE,
    background_software TEXT,
    background_hardware TEXT,
    skill_level VARCHAR(50),
    created_at TIMESTAMP
);

-- Qdrant Collections for RAG
- book_embeddings: Vector embeddings of all book content
- chapter_embeddings: Chapter-wise embeddings
- code_examples: Embeddings for code snippets
🚀 Getting Started
Prerequisites
Node.js 18+ and npm

Python 3.10+

Git

OpenAI API key

Qdrant Cloud account

Neon Postgres database

Installation
Clone the Repository

bash
git clone https://github.com/madnan-github/learn-ai.git
cd learn-ai
Install Frontend Dependencies

bash
npm install
Install Backend Dependencies

bash
cd backend
pip install -r requirements.txt
Configure Environment Variables

bash
# .env file
OPENAI_API_KEY=your_api_key_here
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_key
DATABASE_URL=your_neon_postgres_url
BETTER_AUTH_SECRET=your_auth_secret
Run Development Servers

bash
# Frontend (in one terminal)
npm start

# Backend (in another terminal)
cd backend
uvicorn main:app --reload
📊 Features Implementation Status
Feature	Status	Bonus Points
Docusaurus Book	✅ Complete	Base
RAG Chatbot	✅ Implemented	Base
Text Selection	✅ Working	Base
Better-auth Integration	✅ Complete	+50
Content Personalization	✅ Implemented	+50
Urdu Translation	✅ Working	+50
Claude Subagents	✅ Created	+50
Total Bonus Points	✅ 200/200	+200
🎓 Learning Outcomes
Upon completing this textbook, students will be able to:

Understand Physical AI principles and embodied intelligence

Master ROS 2 for robotic control and simulation

Simulate robots with Gazebo and Unity

Develop with NVIDIA Isaac AI robot platform

Design humanoid robots for natural interactions

Integrate GPT models for conversational robotics

Deploy complete Physical AI systems from simulation to real-world

🔧 Development Workflow
Phase 1: Foundation (Week 1)
Set up Docusaurus with Spec-Kit Plus

Design database schema

Implement authentication system

Phase 2: Core Content (Week 2)
Write Modules 1 & 2 (ROS 2, Gazebo/Unity)

Implement basic RAG chatbot

Add code examples and simulations

Phase 3: Advanced Features (Week 3)
Write Modules 3 & 4 (NVIDIA Isaac, VLA)

Enhance RAG with text selection

Implement personalization engine

Add Urdu translation

Phase 4: Integration & Polish (Week 4)
Complete capstone project

Test all features

Deploy to GitHub Pages

Create demo video

🧪 Testing Strategy
Automated Tests
bash
# Run test suite
cd backend
pytest tests/

# Test categories
- RAG chatbot accuracy (>85% target)
- Personalization engine
- Translation quality
- Authentication flow
- Code example execution
Manual Testing
Cross-browser compatibility

Mobile responsiveness

Performance optimization

User experience flow

📈 Success Metrics
Functional Metrics
Chatbot response accuracy: >85%

Personalization coverage: 100% chapters

Translation completeness: 100% core content

Authentication success rate: >99%

Performance Metrics
Page load time: <3 seconds

Chatbot response time: <5 seconds

Translation latency: <10 seconds

Uptime: >99.5%

🤝 Contributing
This is a hackathon submission, but contributions are welcome! Please:

Fork the repository

Create a feature branch

Make your changes

Submit a pull request

📄 License
This project is licensed under the MIT License - see the LICENSE file for details.

🙏 Acknowledgments
Panaversity for organizing the hackathon

Claude Code for AI-powered development assistance

Spec-Kit Plus for structured book creation

All open-source tools and libraries used in this project