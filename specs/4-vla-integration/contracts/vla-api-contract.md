# VLA System API Contract

## Overview
This document defines the API contracts for the Vision-Language-Action (VLA) system components that facilitate communication between the voice recognition, LLM planning, and action execution modules.

## Voice Recognition Service

### POST /transcribe
Transcribes audio input to text using OpenAI Whisper

**Request**:
```
Content-Type: audio/wav
Headers: Authorization: Bearer {api_key}
Body: Raw audio data
```

**Response**:
```
Status: 200 OK
Content-Type: application/json
{
  "transcription": "string - The transcribed text",
  "confidence": "number - Confidence score (0.0-1.0)",
  "duration": "number - Audio duration in seconds",
  "language": "string - Detected language code"
}
```

**Error Responses**:
- 400: Invalid audio format
- 401: Invalid API key
- 500: Transcription service error

## LLM Planning Service

### POST /generate-plan
Generates an action plan from natural language command and context

**Request**:
```
Content-Type: application/json
Headers: Authorization: Bearer {api_key}
{
  "command": "string - Natural language command",
  "context": "object - Robot and environment context",
  "user_preferences": "object - Optional user preferences"
}
```

**Response**:
```
Status: 200 OK
Content-Type: application/json
{
  "plan_id": "string - Unique identifier for the plan",
  "steps": [
    {
      "action": "string - Action type (NAVIGATE, GRASP, etc.)",
      "parameters": "object - Action-specific parameters",
      "description": "string - Human-readable description",
      "expected_duration": "number - Estimated seconds for step"
    }
  ],
  "estimated_total_duration": "number - Estimated seconds for full plan",
  "confidence": "number - Plan confidence score (0.0-1.0)"
}
```

**Error Responses**:
- 400: Invalid command format
- 401: Invalid API key
- 422: Unprocessable command (impossible action)
- 500: LLM service error

## Action Execution Service

### POST /execute-plan
Executes a generated action plan in the simulation environment

**Request**:
```
Content-Type: application/json
{
  "plan_id": "string - ID of the plan to execute",
  "steps": "array - Action steps to execute",
  "execution_context": "object - Current robot and environment state"
}
```

**Response**:
```
Status: 200 OK
Content-Type: application/json
{
  "execution_id": "string - Unique execution identifier",
  "status": "string - Execution status (pending, executing, completed, failed)",
  "estimated_completion": "number - Seconds until completion",
  "execution_log": "array - Log of executed steps"
}
```

### GET /execution-status/{execution_id}
Retrieves the current status of an action plan execution

**Response**:
```
Status: 200 OK
Content-Type: application/json
{
  "execution_id": "string",
  "status": "string - Current execution status",
  "completed_steps": "number - Count of completed steps",
  "total_steps": "number - Total steps in plan",
  "progress": "number - Percentage complete (0-100)",
  "current_step": "object - Details of currently executing step",
  "error": "string - Error message if execution failed"
}
```

## Vision Processing Service

### POST /detect-objects
Detects objects in an image using computer vision

**Request**:
```
Content-Type: image/jpeg
Body: Image data from robot camera
```

**Response**:
```
Status: 200 OK
Content-Type: application/json
{
  "detections": [
    {
      "object_id": "string - Unique object identifier",
      "class": "string - Object class name",
      "confidence": "number - Detection confidence (0.0-1.0)",
      "bbox": {
        "x": "number - X coordinate of top-left corner",
        "y": "number - Y coordinate of top-left corner",
        "width": "number - Bounding box width",
        "height": "number - Bounding box height"
      },
      "position_3d": {
        "x": "number - X coordinate in 3D space",
        "y": "number - Y coordinate in 3D space",
        "z": "number - Z coordinate in 3D space"
      }
    }
  ],
  "timestamp": "string - ISO 8601 timestamp"
}
```

## Integration Service

### POST /vla-integration/process-command
Main entry point for the VLA system that processes a voice command end-to-end

**Request**:
```
Content-Type: application/json
{
  "audio_data": "string - Base64 encoded audio data",
  "user_context": "object - Information about the user",
  "environment_context": "object - Current environment state"
}
```

**Response**:
```
Status: 200 OK
Content-Type: application/json
{
  "command_id": "string - Unique identifier for the command",
  "transcription": "string - Transcribed text",
  "action_plan": "object - Generated action plan",
  "execution_status": "string - Initial execution status",
  "estimated_duration": "number - Total estimated time in seconds"
}
```

## Error Response Format

All error responses follow this format:
```
{
  "error": {
    "code": "string - Error code",
    "message": "string - Human-readable error message",
    "details": "object - Optional error details",
    "timestamp": "string - ISO 8601 timestamp"
  }
}
```

## Common Error Codes

- `INVALID_INPUT`: Provided data doesn't match expected format
- `SERVICE_UNAVAILABLE`: Required service is not responding
- `INSUFFICIENT_PRIVILEGES`: Missing permissions or API keys
- `RESOURCE_NOT_FOUND`: Requested resource doesn't exist
- `VALIDATION_ERROR`: Request data fails validation rules
- `INTERNAL_ERROR`: Unexpected internal system error