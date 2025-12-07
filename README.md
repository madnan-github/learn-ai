markdown:sp.constitution
Project: AI-Native Textbook on Physical AI & Humanoid Robotics
Core principles:
- Academic rigor with industry relevance
- Hands-on, project-based learning approach
- Accessibility across skill levels (beginners to advanced)
- Integration of theoretical concepts with practical implementation
Key standards:
- All technical concepts must be traceable to official documentation or peer-reviewed
sources
- Citation format: APA style for academic references
- Source types: Mix of peer-reviewed papers, official documentation (ROS 2, NVIDIA), and
industry best practices
- Code examples: Must be executable with clear setup instructions
- Visual content: Include diagrams, screenshots, and simulation visualizations
- Language clarity: Flesch-Kincaid grade 12-14 (university level)
- All claims about hardware performance must be verified with manufacturer specifications
Constraints:
- Book length: 15-20 chapters, 300-400 pages equivalent
- Minimum 50 code examples with working implementations
- Must include assessment questions and project ideas for each module
- Format: Docusaurus markdown with interactive elements
- Timeline: Hackathon submission by Nov 30, 2025
Success criteria:
- Complete Docusaurus book deployed to GitHub Pages
- Integrated RAG chatbot answering questions about book content
- All code examples tested and verified
- Clear learning progression from basics to capstone project
__________________________________________________________________
markdown:sp.specify
Target audience: University students, professionals transitioning to robotics, educators
teaching AI/robotics courses
Focus: Practical implementation of Physical AI systems using ROS 2, Gazebo, NVIDIA Isaac,
and humanoid robotics
Success criteria:
- Readers can set up complete Physical AI development environment
- Implements 4+ working simulations across different modules
- Includes complete capstone project with voice-controlled humanoid
- RAG chatbot successfully answers technical questions from book content
- Personalization features adapt content based on user background
- Translation feature to Urdu works seamlessly
Constraints:
- Must use Spec-Kit Plus and Claude Code for development
- Must implement RAG chatbot with OpenAI Agents/ChatKit SDKs
- Database: Neon Serverless Postgres + Qdrant Cloud Free Tier
- Authentication: Better-auth with user profiling
- Deployment: GitHub Pages for book, separate deployment for chatbot backend
- Timeline: Complete by Nov 30, 2025, 6:00 PM
Features to implement:
1. Book with 4 modules covering course curriculum
2. RAG chatbot with text selection capability
3. User authentication with background profiling
4. Content personalization per chapter
5. Urdu translation feature
6. Reusable subagents and agent skills (for bonus points)
Not building:
- Full commercial robot manufacturing guide
- Deep dive into single vendor's proprietary SDK
- Complete physics engine implementation
- Hardware manufacturing instructions
__________________________________________________________
markdown:sp.clarify
Please analyze the hackathon requirements for:
1. Ambiguous terms:
 - What constitutes "reusable intelligence" for bonus points?
 - How should "personalize the content" be implemented? What parameters?
 - What level of Urdu translation quality is expected?
 - What defines "base functionality" vs "bonus features"?
2. Missing assumptions:
 - Target reader's existing knowledge level?
 - Balance between theory and practical content?
 - Assessment structure and grading?
 - How to handle hardware limitations for readers?
3. Incomplete requirements:
 - How many chapters/sections minimum?
 - What interactive elements beyond chatbot?
 - How to validate RAG chatbot accuracy?
 - What deployment platform for backend services?
4. Scope conflicts:
 - Book vs course platform - which is primary?
 - Academic textbook vs practical tutorial balance?
 - Hardware requirements vs accessibility concerns?
 - Cloud vs local setup emphasis?
Key gaps to address:
- Define clear success metrics for each bonus feature
- Specify minimum content coverage for each module
- Determine authentication flow and data storage
- Plan chatbot training and evaluation methodology
- Establish content personalization rules engine
_________________________________________________________
markdown:sp.plan
Create: Complete textbook architecture, chapter structure, implementation roadmap
Phase 1: Foundation (Week 1)
- Set up Docusaurus project with Spec-Kit Plus
- Create book structure: 4 modules, 15 chapters
- Design database schema for user profiles and interactions
- Set up authentication with Better-auth
Phase 2: Core Content (Week 2)
- Write Module 1: Robotic Nervous System (ROS 2)
- Write Module 2: Digital Twin (Gazebo & Unity)
- Implement basic RAG chatbot with book content
- Add code examples and simulation instructions
Phase 3: Advanced Features (Week 3)
- Write Module 3: AI-Robot Brain (NVIDIA Isaac)
- Write Module 4: Vision-Language-Action (VLA)
- Enhance RAG with text selection capability
- Implement personalization engine
- Add Urdu translation feature
Phase 4: Integration & Polish (Week 4)
- Complete capstone project documentation
- Integrate all bonus features
- Test chatbot accuracy
- Deploy to GitHub Pages
- Prepare demo video
Technical Architecture:
- Frontend: Docusaurus (GitHub Pages)
- Backend: FastAPI (Vercel/Heroku)
- Database: Neon Postgres (user data) + Qdrant (embeddings)
- AI: OpenAI API for chatbot + translation
- Auth: Better-auth with custom profile fields
Decisions needing documentation:
1. Content depth vs breadth - Option: Focus on implementable projects
2. Hardware emphasis - Option: Provide both cloud and local paths
3. Assessment approach - Option: Project-based with code submissions
4. Translation quality - Option: Professional translation for key terms only
Testing strategy:
- RAG chatbot: Test with 50+ questions from each module
- Personalization: Verify content changes based on 3+ profile types
- Translation: Validate technical terms maintain meaning
- Code examples: Test in clean environment setups
- Deployment: Verify all features work in production
Quality validation:
- Peer review of technical accuracy
- Student-level testing of tutorials
- Performance testing of chatbot responses
- Cross-browser compatibility check
- Mobile responsiveness verification
________________________________________________________
markdown:sp.implementation
Project: Physical AI & Humanoid Robotics Textbook with AI-Native Features
TECHNICAL ARCHITECTURE
1. Docusaurus Book Structure:
/book
├── /docs
│ ├── /module-1-ros2
│ ├── /module-2-gazebo-unity
│ ├── /module-3-nvidia-isaac
│ └── /module-4-vla
├── /src
│ ├── /components
│ │ ├── ChatbotWidget.jsx
│ │ ├── PersonalizeButton.jsx
│ │ ├── TranslateButton.jsx
│ │ └── UserProfile.jsx
│ └── /pages
├── /static
│ ├── /images
│ └── /videos
├── docusaurus.config.js
└── package.json
2. Backend Services Architecture:
/backend
├── /api
│ ├── auth.py (Better-auth integration)
│ ├── chatbot.py (RAG implementation)
│ ├── personalize.py (Content adaptation)
│ └── translate.py (Urdu translation)
├── /models
│ ├── embeddings.py (Qdrant integration)
│ └── database.py (Neon Postgres models)
├── /utils
│ ├── rag_engine.py
│ └── content_parser.py
└── main.py (FastAPI app)
3. Database Schema:
```sql
-- Neon Postgres Tables
CREATE TABLE users (
 id UUID PRIMARY KEY,
 email VARCHAR(255) UNIQUE,
 background_software TEXT,
 background_hardware TEXT,
 skill_level VARCHAR(50),
 created_at TIMESTAMP
);
CREATE TABLE user_preferences (
 user_id UUID REFERENCES users(id),
 chapter_id VARCHAR(100),
 personalized_content TEXT,
 translation_preference VARCHAR(10),
 PRIMARY KEY (user_id, chapter_id)
);
CREATE TABLE chat_history (
 id SERIAL PRIMARY KEY,
 user_id UUID REFERENCES users(id),
 question TEXT,
 answer TEXT,
 context_text TEXT,
 timestamp TIMESTAMP
);
-- Qdrant Collections for RAG
- book_embeddings: Stores vector embeddings of all book content
- chapter_embeddings: Chapter-wise embeddings for better retrieval
- code_examples: Embeddings for code snippets
IMPLEMENTATION DETAILS
1. RAG Chatbot Implementation:
Python code
class TextbookRAG: def init(self): self.qdrant_client =
QdrantClient(url="cloud.qdrant.io") self.embedder = OpenAIEmbeddings(model="textembedding-3-small") self.llm = ChatOpenAI(model="gpt-4o-mini")
async def answer_question(self, question: str, selected_text: str = None):
 # If text is selected, use it as primary context
 if selected_text:
 context = self._enhance_with_selected_text(selected_text, question)
 else:
 # Search in Qdrant for relevant content
 context = self._search_qdrant(question)

 # Generate answer using context
 prompt = self._create_prompt(context, question)
 return await self.llm.ainvoke(prompt)
def _search_qdrant(self, query: str, limit: int = 5):
 query_embedding = self.embedder.embed_query(query)
 results = self.qdrant_client.search(
 collection_name="book_embeddings",
 query_vector=query_embedding,
 limit=limit
 )
 return "\n".join([r.payload["content"] for r in results])
2. Personalization Engine:
class ContentPersonalizer: PERSONALIZATION_RULES = { "beginner":
{ "add_explanations": True, "simplify_terminology": True, "include_basics": True,
"extra_examples": 3 }, "intermediate": { "add_explanations": False, "simplify_terminology":
False, "include_basics": False, "extra_examples": 1 }, "expert": { "add_explanations": False,
"simplify_terminology": False, "include_basics": False, "extra_examples": 0,
"add_advanced_topics": True } }
def personalize_content(self, original_content: str,
user_profile: dict):
 rules = self.PERSONALIZATION_RULES.get(
 user_profile.get("skill_level", "beginner"),
 self.PERSONALIZATION_RULES["beginner"]
 )

 personalized = original_content
 if rules["add_explanations"]:
 personalized = self._add_explanations(personalized)
 if rules["simplify_terminology"]:
 personalized = self._simplify_terms(personalized)

 return personalized
3. Urdu Translation Service:
// Frontend Translation Component
class UrduTranslator {
 constructor() {
 this.translationCache = new Map();
 }

 async translateChapter(chapterId, content) {
 // Check cache first
 if (this.translationCache.has(chapterId)) {
 return this.translationCache.get(chapterId);
 }

 // API call for translation
 const response = await fetch('/api/translate', {
 method: 'POST',
 headers: { 'Content-Type': 'application/json' },
 body: JSON.stringify({
 text: content,
 target_lang: 'ur'
 })
 });

 const translated = await response.json();
 this.translationCache.set(chapterId, translated.text);
 return translated.text;
 }

 // Toggle translation in UI
 toggleTranslation(chapterElement, isTranslated) {
 if (isTranslated) {
 chapterElement.classList.add('urdu-text');
 chapterElement.dir = 'rtl';
 chapterElement.style.fontFamily = "'Noto Nastaliq Urdu', serif';
 } else {
 chapterElement.classList.remove('urdu-text');
 chapterElement.dir = 'ltr';
 chapterElement.style.fontFamily = '';
 }
 }
}
4. 4.Claude Code Subagents Structure:
/subagents ├── technical_writer_agent │ ├── spec.md │ ├── skills.py │ └── examples/
├── code_review_agent │ ├── spec.md │ ├── skills.py │ └── templates/ ├──
content_validator_agent │ ├── spec.md │ ├── skills.py │ └── rules/ └──
deployment_agent ├── spec.md ├── skills.py └── workflows/
DEPENDENCIES & SETUP
1. Frontend (Docusaurus):
{
 "dependencies": {
 "@docusaurus/core": "^3.0.0",
 "@docusaurus/preset-classic": "^3.0.0",
 "react": "^18.0.0",
 "react-dom": "^18.0.0",
 "axios": "^1.6.0",
 "@better-auth/react": "^1.0.0",
 "prism-react-renderer": "^2.0.0"
 }
}
2. Backend (FastAPI):
fastapi==0.104.0
uvicorn==0.24.0
openai==1.3.0
qdrant-client==1.6.0
asyncpg==0.29.0
better-auth==0.1.0
python-multipart==0.0.6
3. Deployment Configuration:
GitHub Actions Workflow
# GitHub Actions Workflow
name: Deploy Book
on:
 push:
 branches: [main]
jobs:
 deploy:
 runs-on: ubuntu-latest
 steps:
 - uses: actions/checkout@v3
 - uses: actions/setup-node@v3
 - run: npm install
 - run: npm run build
 - uses: peaceiris/actions-gh-pages@v3
 with:
 github_token: ${{ secrets.GITHUB_TOKEN }}
 publish_dir: ./build
DEVELOPMENT WORKFLOW
Research → Write → Add Code Examples → Review → Personalize → Translate
 ↓ ↓ ↓ ↓ ↓ ↓
Claude Code → Validation → Testing → Peer Review → A/B Test → Publish
2. Testing Strategy:
# Test suite structure
/tests
 ├── test_rag_chatbot.py
 ├── test_personalization.py
 ├── test_translation.py
 ├── test_integration.py
 └── test_deployment.py
3. Validation Checks:
• RAG chatbot accuracy > 85% on test questions
• Personalization works for all 3 skill levels
• Urdu translation maintains technical accuracy
• All code examples execute without errors
• Authentication flow complete
• Mobile responsive design
• Load time < 3 seconds for all pages
IMPLEMENTATION TIMELINE
Week 1 (Nov 1-7):
• Set up Docusaurus project
• Configure Spec-Kit Plus
• Design database schema
• Implement basic authentication
Week 2 (Nov 8-14):
• Write Module 1 & 2 content
• Implement RAG chatbot core
• Set up Qdrant embeddings
• Create personalization engine
Week 3 (Nov 15-21):
• Write Module 3 & 4 content
• Implement Urdu translation
• Add Claude subagents
• Test all integrations
Week 4 (Nov 22-30):
• Polish and refine content
• Performance optimization
• Create demo video
• Final deployment
RISK MITIGATION
1. Technical Risks:
a. Qdrant Cloud limitations → Implement local fallback with FAISS
b. OpenAI API costs → Implement caching and rate limiting
c. Deployment issues → Use multiple deployment platforms (Vercel + Railway)
2. Content Risks:
a. Technical inaccuracies → Peer review process with domain experts
b. Code compatibility → Test on multiple environments
c. Translation quality → Professional review of technical terms
3. Timeline Risks:
a. Over-engineering → Focus on MVP first, then add features
b. Integration complexity → Use modular, decoupled architecture
c. Testing delays → Implement automated testing from day 1
SUCCESS METRICS
1. Functional Metrics:
a. Chatbot response accuracy: > 85%
b. Personalization coverage: 100% chapters
c. Translation completeness: 100% core content
d. Authentication success rate: > 99%
2. Performance Metrics:
a. Page load time: < 3s
b. Chatbot response time: < 5s
c. Translation latency: < 10s
d. Uptime: > 99.5%
3. Quality Metrics:
a. Code example execution success: 100%
b. Broken links: 0
c. Mobile responsiveness: Perfect score
d. Accessibility compliance: WCAG 2.1 AA
_____________________________________
Yeh `/sp.implementation` file aapko complete technical blueprint provide karti hai
jismein:
1. **Complete architecture** with folder structures
2. **Code examples** for key components
3. **Database schemas** for all features
4. **Dependencies** and setup instructions
5. **Development workflow** with timeline
6. **Testing strategy** and validation checks
7. **Risk mitigation** plans
8. **Success metrics** for evaluation
Is file ko use karke aap directly implementation start kar sakte hain, aur yeh
aapko hackathon ke saare requirements ko systematically address karne mein madad
karegi.