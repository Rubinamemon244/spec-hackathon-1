# Quickstart: Vision-Language-Action (VLA) Module

## Prerequisites

Before starting with the VLA module, ensure you have:

### System Requirements
- Computer with NVIDIA GPU (RTX series recommended)
- At least 16GB RAM (32GB recommended)
- Ubuntu 22.04 LTS or Windows with WSL2
- ROS2 Humble Hawksbill installed
- NVIDIA Isaac Sim 2023.1+ (requires NVIDIA GPU drivers)
- Isaac ROS packages
- Nav2 navigation stack

### Software Dependencies
- Python 3.8+
- Node.js 16+ (for Docusaurus)
- OpenAI API access or local Whisper installation
- Large Language Model access (OpenAI GPT, Anthropic Claude, or local models)

## Setup Instructions

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd hackathon-1
git checkout 4-vla-integration
cd book-frontend
```

### 2. Install Docusaurus Dependencies
```bash
npm install
```

### 3. Set Up Isaac Simulation Environment
```bash
# Follow the Isaac Sim installation guide for your platform
# Ensure Isaac ROS packages are installed
# Verify Nav2 is properly configured
```

### 4. Configure API Keys
```bash
# Create .env file in book-frontend directory
echo "OPENAI_API_KEY=your_openai_api_key_here" > .env
echo "ANTHROPIC_API_KEY=your_anthropic_api_key_here" >> .env
```

## Running the Documentation

### 1. Start Documentation Server
```bash
npm start
```

The documentation will be available at `http://localhost:3000`

### 2. Access VLA Module Content
Navigate to the VLA module:
- Voice-to-Action Interfaces: `/docs/module-4-vla/voice-to-action-interfaces`
- LLM-Based Cognitive Planning: `/docs/module-4-vla/llm-based-cognitive-planning`
- Capstone: The Autonomous Humanoid: `/docs/module-4-vla/capstone-autonomous-humanoid`

## Running the VLA System

### 1. Launch Isaac Simulation
```bash
# Launch Isaac Sim with your humanoid robot configuration
ros2 launch isaac_ros_bringup isaac_sim.launch.py
```

### 2. Start VLA System Components
```bash
# Terminal 1: Start voice recognition
python3 vla_system/voice_component.py

# Terminal 2: Start LLM planning
python3 vla_system/planning_component.py

# Terminal 3: Start action execution
python3 vla_system/action_component.py

# Terminal 4: Start integration coordinator
python3 vla_system/integration_node.py
```

## Basic Usage Examples

### Voice Command Example
1. Speak into the microphone: "Move forward 2 meters"
2. Whisper processes audio and transcribes: "move forward 2 meters"
3. LLM generates action plan: [NAVIGATE_TO_LOCATION with parameters]
4. Robot executes the navigation command in simulation

### Complex Command Example
1. Speak: "Go to the kitchen and bring me the blue cup"
2. System processes through VLA pipeline:
   - Vision: Identifies kitchen location and blue cup
   - Language: Parses command into navigation and manipulation tasks
   - Action: Executes sequence of navigation, detection, grasping, and return

## Troubleshooting Common Issues

### Audio Input Issues
- Check microphone permissions and connection
- Verify audio input levels are appropriate
- Ensure Whisper model is properly loaded

### LLM Communication Issues
- Verify API keys are correctly configured
- Check network connectivity to LLM services
- Monitor API usage limits

### Simulation Integration Issues
- Confirm Isaac Sim is running properly
- Verify ROS 2 network configuration
- Check that all required Isaac ROS packages are active

## Next Steps

1. Complete the Voice-to-Action Interfaces chapter
2. Proceed to LLM-Based Cognitive Planning
3. Integrate components in the Capstone project
4. Test complete VLA pipeline in simulation