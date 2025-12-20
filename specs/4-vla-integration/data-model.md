# Data Model: Vision-Language-Action (VLA) Module

## Core Entities

### VoiceCommand
Represents a spoken command that needs to be processed by the system, containing audio data and transcription results.

**Attributes**:
- `id`: Unique identifier for the command
- `audio_data`: Raw audio input from microphone
- `transcription`: Text result from Whisper speech recognition
- `confidence`: Confidence score for the transcription (0.0-1.0)
- `timestamp`: When the command was received
- `user_context`: Context about the user issuing the command
- `processed`: Boolean indicating if command has been processed

**Validation Rules**:
- Audio data must not be empty
- Confidence must be between 0.0 and 1.0
- Timestamp must be in the past or present

### ActionPlan
Represents a sequence of robot actions generated from natural language, containing individual action steps with parameters and execution order.

**Attributes**:
- `id`: Unique identifier for the action plan
- `command_text`: Original natural language command
- `steps`: Array of action steps to execute
- `status`: Current execution status (pending, executing, completed, failed)
- `created_at`: When the plan was generated
- `estimated_duration`: Estimated time to complete the plan
- `context`: Environmental and robot state context

**Action Step Structure**:
- `action_type`: Type of action (NAVIGATE, GRASP, SPEAK, DETECT_OBJECT, etc.)
- `parameters`: Action-specific parameters
- `description`: Human-readable description of the step
- `expected_duration`: Estimated time for this step

**Validation Rules**:
- Must have at least one action step
- All action types must be valid
- Parameters must match the action type requirements

### VLAComponent
Represents an integrated system component (Vision, Language, Action) that processes specific modalities.

**Attributes**:
- `id`: Unique identifier for the component
- `type`: Component type (VISION, LANGUAGE, ACTION)
- `status`: Operational status (active, inactive, error)
- `configuration`: Component-specific configuration parameters
- `input_format`: Expected input data format
- `output_format`: Output data format
- `last_updated`: When component state was last updated

**Validation Rules**:
- Type must be one of the valid component types
- Status must be valid
- Configuration must be valid JSON

### SimulationEnvironment
Represents the virtual environment where the complete VLA system operates, containing objects, obstacles, and interaction spaces.

**Attributes**:
- `id`: Unique identifier for the environment
- `name`: Human-readable name of the environment
- `description`: Detailed description of the environment
- `objects`: Array of objects present in the environment
- `robot_position`: Current position of the robot in the environment
- `navigation_points`: Valid locations for robot navigation
- `simulation_state`: Current state of the simulation

**Object Structure**:
- `object_id`: Unique identifier for the object
- `type`: Type of object (furniture, tool, person, etc.)
- `position`: 3D position in the environment
- `properties`: Object-specific properties

**Validation Rules**:
- Name must not be empty
- Objects must have valid positions
- Navigation points must be within environment boundaries

## Relationships

### VoiceCommand → ActionPlan
- One VoiceCommand generates one ActionPlan (1:1 relationship)
- When a voice command is processed successfully, it creates a corresponding action plan

### ActionPlan → VLAComponent
- One ActionPlan involves multiple VLAComponents (1:many relationship)
- Action plans are executed using the VLA components

### VLAComponent → SimulationEnvironment
- Multiple VLAComponents operate within one SimulationEnvironment (many:1 relationship)
- Components interact with the environment during execution

## State Transitions

### VoiceCommand States
```
RECEIVED → PROCESSING → RECOGNIZED → PLANNED → EXECUTED
     ↓              ↓           ↓           ↓
   FAILED ←--------- ←--------- ←--------- ←
```

### ActionPlan States
```
PENDING → GENERATING → VALIDATING → EXECUTING → COMPLETED
     ↓         ↓            ↓           ↓           ↓
   FAILED ←---- ←------------ ←--------- ←--------- ←
```

## Data Flow

The data flows through the system as follows:
1. VoiceCommand is created from audio input
2. VoiceCommand is processed by LANGUAGE component to generate ActionPlan
3. ActionPlan is validated and executed using ACTION component
4. ACTION component interacts with SimulationEnvironment
5. Vision component monitors SimulationEnvironment for feedback
6. System state is updated based on execution results