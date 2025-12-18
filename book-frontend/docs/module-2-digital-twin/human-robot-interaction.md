---
sidebar_position: 15
---

# Human-Robot Interaction in Digital Twin Environments

## Learning Objectives

By the end of this section, you will be able to:
- Design environments that facilitate effective human-robot interaction
- Implement social navigation systems for humanoid robots
- Create collaborative spaces for human-robot cooperation
- Understand proxemics and personal space considerations in robotics

## Overview

Human-robot interaction (HRI) is a critical aspect of humanoid robotics that requires specialized consideration in digital twin environments. Unlike industrial robots operating in isolated spaces, humanoid robots must navigate and interact safely and effectively alongside humans. This section explores how to create Unity environments that support realistic human-robot interaction scenarios, including navigation, communication, and collaborative tasks.

## Proxemics and Personal Space

### Understanding Proxemics in Robotics

Proxemics, the study of spatial relationships in human interaction, is fundamental to designing effective human-robot interaction:

- **Intimate Distance (0-45cm)**: Reserved for close relationships, rarely appropriate for human-robot interaction
- **Personal Distance (45-120cm)**: Comfortable for one-on-one conversations and interactions
- **Social Distance (120-360cm)**: Appropriate for casual social interactions and professional settings
- **Public Distance (360cm+)**: Used for public speaking and distant observation

### Personal Space Implementation

```csharp
using UnityEngine;
using System.Collections.Generic;

public class PersonalSpaceManager : MonoBehaviour
{
    [Header("Personal Space Configuration")]
    public float intimateDistance = 0.45f;    // 45cm
    public float personalDistance = 1.2f;     // 1.2m
    public float socialDistance = 3.6f;       // 3.6m
    public float publicDistance = 10.0f;      // 10m

    [Header("Human Agent Settings")]
    public float humanPersonalSpaceRadius = 1.2f;
    public float humanComfortZone = 0.8f;

    [Header("Robot Interaction Parameters")]
    public float robotPersonalSpaceRadius = 1.0f;
    public float robotApproachThreshold = 2.0f;
    public float robotInteractionDistance = 1.5f;

    private Dictionary<Transform, float> personalSpaces = new Dictionary<Transform, float>();
    private List<Transform> humans = new List<Transform>();
    private Transform robotTransform;

    void Start()
    {
        robotTransform = transform; // Assuming this component is on the robot
        SetupPersonalSpaceVisualization();
    }

    void SetupPersonalSpaceVisualization()
    {
        // Create visual indicators for personal space
        CreatePersonalSpaceIndicator(robotTransform, robotPersonalSpaceRadius, Color.red);
        CreatePersonalSpaceIndicator(robotTransform, robotApproachThreshold, Color.yellow);
        CreatePersonalSpaceIndicator(robotTransform, robotInteractionDistance, Color.green);
    }

    GameObject CreatePersonalSpaceIndicator(Transform owner, float radius, Color color)
    {
        GameObject indicator = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        indicator.name = $"{owner.name}_PersonalSpace_{radius}m";
        indicator.transform.position = owner.position + Vector3.up * 0.05f; // Slightly above ground
        indicator.transform.localScale = new Vector3(radius * 2, 0.1f, radius * 2); // Flat circle
        indicator.GetComponent<Renderer>().material.color = color;
        indicator.GetComponent<Renderer>().material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        indicator.GetComponent<Renderer>().material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        indicator.GetComponent<Renderer>().material.renderQueue = 3000; // Transparent queue

        // Make it a trigger (no collision)
        indicator.GetComponent<Collider>().isTrigger = true;

        return indicator;
    }

    void Update()
    {
        CheckPersonalSpaceViolations();
        UpdateApproachBehavior();
    }

    void CheckPersonalSpaceViolations()
    {
        // Check for personal space violations with nearby humans
        Collider[] nearbyHumans = Physics.OverlapSphere(transform.position, socialDistance, LayerMask.GetMask("Human"));

        foreach (Collider human in nearbyHumans)
        {
            float distance = Vector3.Distance(transform.position, human.transform.position);
            Transform humanTransform = human.transform;

            if (distance < humanPersonalSpaceRadius)
            {
                // Too close! Enter respectful behavior mode
                HandlePersonalSpaceViolation(humanTransform, distance);
            }
            else if (distance < humanPersonalSpaceRadius + 0.5f)
            {
                // Approaching personal space boundary
                HandleApproachingPersonalSpace(humanTransform, distance);
            }
        }
    }

    void HandlePersonalSpaceViolation(Transform human, float distance)
    {
        Debug.LogWarning($"Personal space violation with {human.name}. Distance: {distance:F2}m");

        // Implement respectful retreat behavior
        Vector3 retreatDirection = (transform.position - human.position).normalized;
        Vector3 retreatTarget = human.position + retreatDirection * (robotPersonalSpaceRadius + 0.2f);

        // Move to respectful distance
        transform.position = Vector3.MoveTowards(transform.position, retreatTarget, 0.05f);
    }

    void HandleApproachingPersonalSpace(Transform human, float distance)
    {
        Debug.Log($"Approaching personal space of {human.name}. Distance: {distance:F2}m");

        // Slow down approach and indicate intention
        // This could involve animation, speech, or gesture
        GetComponent<Animator>()?.SetBool("Approaching", true);
    }

    void UpdateApproachBehavior()
    {
        // Check if robot should approach humans for interaction
        Collider[] potentialInteractions = Physics.OverlapSphere(transform.position, robotApproachThreshold, LayerMask.GetMask("Human"));

        if (potentialInteractions.Length > 0)
        {
            // Select closest human for interaction
            Transform closestHuman = GetClosestTransform(potentialInteractions);

            if (closestHuman != null)
            {
                float distance = Vector3.Distance(transform.position, closestHuman.position);

                if (distance > robotInteractionDistance)
                {
                    // Move toward interaction distance
                    Vector3 approachDirection = (closestHuman.position - transform.position).normalized;
                    Vector3 approachTarget = closestHuman.position - approachDirection * robotInteractionDistance;

                    // Smooth approach
                    transform.position = Vector3.MoveTowards(transform.position, approachTarget, 0.02f);
                }
            }
        }
    }

    Transform GetClosestTransform(Collider[] colliders)
    {
        Transform closest = null;
        float minDistance = float.MaxValue;

        foreach (Collider col in colliders)
        {
            float dist = Vector3.Distance(transform.position, col.transform.position);
            if (dist < minDistance)
            {
                minDistance = dist;
                closest = col.transform;
            }
        }

        return closest;
    }
}
```

## Social Navigation in Crowded Environments

### Navigation Mesh Setup for Humanoid Robots

```csharp
using UnityEngine;
using UnityEngine.AI;

public class SocialNavigationAgent : MonoBehaviour
{
    [Header("Navigation Configuration")]
    public NavMeshAgent navAgent;
    public float agentRadius = 0.4f;        // Half the width of the robot
    public float agentHeight = 1.7f;        // Height of the robot
    public float maxSpeed = 1.0f;           // Walking speed
    public float maxAcceleration = 8.0f;    // Acceleration limit

    [Header("Social Behavior Parameters")]
    public float comfortableDistance = 1.5f;  // Distance to maintain from humans
    public float interactionDistance = 1.0f;  // Distance for direct interaction
    public float crowdDensityThreshold = 0.5f; // When to slow down in crowds

    [Header("Communication Signals")]
    public bool enableVisualSignals = true;
    public bool enableAuditorySignals = true;

    private Animator animator;
    private List<GameObject> nearbyHumans = new List<GameObject>();

    void Start()
    {
        SetupNavigationAgent();
        SetupSocialBehaviors();
    }

    void SetupNavigationAgent()
    {
        if (navAgent == null)
        {
            navAgent = GetComponent<NavMeshAgent>();
        }

        if (navAgent == null)
        {
            navAgent = gameObject.AddComponent<NavMeshAgent>();
        }

        // Configure agent properties
        navAgent.radius = agentRadius;
        navAgent.height = agentHeight;
        navAgent.speed = maxSpeed;
        navAgent.acceleration = maxAcceleration;
        navAgent.angularSpeed = 120f; // Degrees per second
        navAgent.stoppingDistance = interactionDistance;
        navAgent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
    }

    void SetupSocialBehaviors()
    {
        animator = GetComponent<Animator>();

        // Set up triggers for social behaviors
        if (animator != null)
        {
            animator.SetLayerWeight(1, 1.0f); // Weight for social behavior animations
        }
    }

    void Update()
    {
        DetectNearbyHumans();
        UpdateSocialNavigation();
        HandleCrowdBehavior();
    }

    void DetectNearbyHumans()
    {
        nearbyHumans.Clear();

        Collider[] humans = Physics.OverlapSphere(transform.position, comfortableDistance, LayerMask.GetMask("Human"));

        foreach (Collider human in humans)
        {
            nearbyHumans.Add(human.gameObject);
        }
    }

    void UpdateSocialNavigation()
    {
        if (!navAgent.pathPending && navAgent.remainingDistance < navAgent.stoppingDistance)
        {
            // Reached destination - check for interaction opportunities
            CheckForInteractionOpportunities();
        }
        else if (nearbyHumans.Count > 0)
        {
            // Adjust navigation based on nearby humans
            AdjustPathForSocialConsiderations();
        }
    }

    void AdjustPathForSocialConsiderations()
    {
        // Modify path to respect personal space
        Vector3 originalDestination = navAgent.destination;

        if (nearbyHumans.Count > 0)
        {
            // Calculate repulsion forces from nearby humans
            Vector3 repulsionForce = CalculateRepulsionForce();

            // Adjust destination slightly to maintain comfortable distance
            Vector3 adjustedDestination = originalDestination + repulsionForce * 0.1f;

            // Ensure adjusted destination is on NavMesh
            NavMeshHit hit;
            if (NavMesh.SamplePosition(adjustedDestination, out hit, 2.0f, NavMesh.AllAreas))
            {
                navAgent.SetDestination(hit.position);
            }
        }
    }

    Vector3 CalculateRepulsionForce()
    {
        Vector3 totalForce = Vector3.zero;

        foreach (GameObject human in nearbyHumans)
        {
            Vector3 directionToHuman = transform.position - human.transform.position;
            float distance = directionToHuman.magnitude;

            if (distance > 0.1f) // Avoid division by zero
            {
                directionToHuman.Normalize();

                // Repulsion force decreases with distance (inverse square law)
                float forceMagnitude = 1.0f / (distance * distance);
                totalForce += directionToHuman * forceMagnitude;
            }
        }

        return totalForce.normalized * Mathf.Clamp01(totalForce.magnitude);
    }

    void HandleCrowdBehavior()
    {
        // Detect crowded areas and adjust behavior
        Collider[] nearbyAgents = Physics.OverlapSphere(transform.position, comfortableDistance * 2,
            LayerMask.GetMask("Human") | LayerMask.GetMask("Robot"));

        float density = (float)nearbyAgents.Length / (Mathf.PI * Mathf.Pow(comfortableDistance * 2, 2));

        if (density > crowdDensityThreshold)
        {
            // In dense crowd - slow down and be more cautious
            navAgent.speed = maxSpeed * 0.5f;
            navAgent.acceleration = maxAcceleration * 0.5f;

            // Play cautious animation
            if (animator != null)
            {
                animator.SetBool("Cautious", true);
                animator.SetBool("Normal", false);
            }
        }
        else
        {
            // Normal speed
            navAgent.speed = maxSpeed;
            navAgent.acceleration = maxAcceleration;

            if (animator != null)
            {
                animator.SetBool("Cautious", false);
                animator.SetBool("Normal", true);
            }
        }
    }

    void CheckForInteractionOpportunities()
    {
        // Check if there are humans nearby for interaction
        if (nearbyHumans.Count > 0)
        {
            GameObject closestHuman = GetClosestHuman();

            if (closestHuman != null)
            {
                float distance = Vector3.Distance(transform.position, closestHuman.transform.position);

                if (distance <= interactionDistance)
                {
                    // Close enough for interaction - initiate interaction protocol
                    InitiateInteraction(closestHuman);
                }
            }
        }
    }

    GameObject GetClosestHuman()
    {
        GameObject closest = null;
        float minDistance = float.MaxValue;

        foreach (GameObject human in nearbyHumans)
        {
            float distance = Vector3.Distance(transform.position, human.transform.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                closest = human;
            }
        }

        return closest;
    }

    void InitiateInteraction(GameObject human)
    {
        Debug.Log($"Initiating interaction with {human.name}");

        // Stop navigation temporarily
        navAgent.isStopped = true;

        // Play greeting animation
        if (animator != null)
        {
            animator.SetTrigger("Greet");
        }

        // Simulate interaction delay
        Invoke("ResumeNavigation", 2.0f);
    }

    void ResumeNavigation()
    {
        navAgent.isStopped = false;
    }
}
```

## Collaborative Task Environments

### Designing Cooperative Spaces

```csharp
using UnityEngine;
using System.Collections.Generic;

public class CollaborativeWorkspace : MonoBehaviour
{
    [Header("Workspace Configuration")]
    public Transform[] collaborationZones;     // Areas designed for human-robot collaboration
    public Transform[] taskStations;           // Specific locations for tasks
    public Transform[] toolPositions;          // Locations for shared tools/resources

    [Header("Collaboration Parameters")]
    public float collaborationRadius = 2.0f;   // Radius around each zone
    public float optimalDistance = 1.2f;       // Best distance for collaboration
    public float toolSharingDistance = 0.8f;   // Distance for tool sharing

    [Header("Task Management")]
    public List<CollaborationTask> availableTasks = new List<CollaborationTask>();
    public List<ActiveCollaboration> activeCollaborations = new List<ActiveCollaboration>();

    [Header("Communication Channels")]
    public bool enableVisualFeedback = true;
    public bool enableAudioFeedback = true;
    public bool enableGestureCommunication = true;

    [System.Serializable]
    public class CollaborationTask
    {
        public string taskName;
        public Vector3 taskLocation;
        public int requiredHumans = 1;
        public int requiredRobots = 1;
        public float estimatedDuration = 30.0f; // seconds
        public bool isAvailable = true;
    }

    [System.Serializable]
    public class ActiveCollaboration
    {
        public CollaborationTask task;
        public List<GameObject> humanParticipants = new List<GameObject>();
        public List<GameObject> robotParticipants = new List<GameObject>();
        public float startTime;
        public float progress = 0.0f;
    }

    void Start()
    {
        SetupCollaborationZones();
        InitializeTaskSystem();
    }

    void SetupCollaborationZones()
    {
        foreach (Transform zone in collaborationZones)
        {
            // Create visual indicator for collaboration zone
            GameObject zoneIndicator = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            zoneIndicator.name = $"CollabZone_{zone.name}";
            zoneIndicator.transform.position = zone.position + Vector3.up * 0.01f; // Slightly above ground
            zoneIndicator.transform.localScale = new Vector3(collaborationRadius * 2, 0.02f, collaborationRadius * 2);
            zoneIndicator.GetComponent<Renderer>().material.color = new Color(0, 1, 0, 0.3f); // Transparent green
            zoneIndicator.GetComponent<Collider>().isTrigger = true;

            // Add zone-specific collider for detection
            SphereCollider sphereCollider = zoneIndicator.AddComponent<SphereCollider>();
            sphereCollider.isTrigger = true;
            sphereCollider.radius = collaborationRadius;
        }
    }

    void InitializeTaskSystem()
    {
        // Create some example tasks
        CreateExampleTasks();
    }

    void CreateExampleTasks()
    {
        // Example: Assembly task
        CollaborationTask assemblyTask = new CollaborationTask
        {
            taskName = "Assembly Task",
            taskLocation = taskStations.Length > 0 ? taskStations[0].position : Vector3.zero,
            requiredHumans = 1,
            requiredRobots = 1,
            estimatedDuration = 120.0f, // 2 minutes
            isAvailable = true
        };
        availableTasks.Add(assemblyTask);

        // Example: Sorting task
        CollaborationTask sortingTask = new CollaborationTask
        {
            taskName = "Sorting Task",
            taskLocation = taskStations.Length > 1 ? taskStations[1].position : Vector3.one,
            requiredHumans = 1,
            requiredRobots = 1,
            estimatedDuration = 90.0f, // 1.5 minutes
            isAvailable = true
        };
        availableTasks.Add(sortingTask);
    }

    void Update()
    {
        ManageActiveCollaborations();
        HandleTaskAssignment();
    }

    void ManageActiveCollaborations()
    {
        // Update progress for active collaborations
        for (int i = activeCollaborations.Count - 1; i >= 0; i--)
        {
            ActiveCollaboration collab = activeCollaborations[i];

            if (collab.task != null)
            {
                // Calculate progress based on time elapsed
                float elapsedTime = Time.time - collab.startTime;
                collab.progress = Mathf.Clamp01(elapsedTime / collab.task.estimatedDuration);

                // Check if task is complete
                if (collab.progress >= 1.0f)
                {
                    CompleteCollaboration(collab);
                    activeCollaborations.RemoveAt(i);
                }
                else
                {
                    // Update participants on progress
                    UpdateCollaborationProgress(collab);
                }
            }
        }
    }

    void HandleTaskAssignment()
    {
        // Look for available humans and robots near collaboration zones
        Collider[] nearbyHumans = Physics.OverlapSphere(transform.position, collaborationRadius * 3, LayerMask.GetMask("Human"));
        Collider[] nearbyRobots = Physics.OverlapSphere(transform.position, collaborationRadius * 3, LayerMask.GetMask("Robot"));

        if (nearbyHumans.Length > 0 && nearbyRobots.Length > 0)
        {
            // Check for available tasks
            CollaborationTask availableTask = GetAvailableTask();

            if (availableTask != null)
            {
                // Form collaboration
                FormNewCollaboration(availableTask, nearbyHumans[0].gameObject, nearbyRobots[0].gameObject);
            }
        }
    }

    CollaborationTask GetAvailableTask()
    {
        foreach (CollaborationTask task in availableTasks)
        {
            if (task.isAvailable)
            {
                return task;
            }
        }
        return null;
    }

    void FormNewCollaboration(CollaborationTask task, GameObject human, GameObject robot)
    {
        ActiveCollaboration newCollab = new ActiveCollaboration
        {
            task = task,
            startTime = Time.time,
            progress = 0.0f
        };

        newCollab.humanParticipants.Add(human);
        newCollab.robotParticipants.Add(robot);

        activeCollaborations.Add(newCollab);

        // Mark task as unavailable
        task.isAvailable = false;

        // Notify participants
        NotifyParticipantsOfCollaboration(newCollab);

        Debug.Log($"New collaboration formed: {task.taskName} with {human.name} and {robot.name}");
    }

    void NotifyParticipantsOfCollaboration(ActiveCollaboration collab)
    {
        // Send notifications to human and robot participants
        foreach (GameObject human in collab.humanParticipants)
        {
            // In a real system, this would trigger UI or audio notification
            Debug.Log($"{human.name} notified of collaboration: {collab.task.taskName}");
        }

        foreach (GameObject robot in collab.robotParticipants)
        {
            // Send task assignment to robot
            SocialNavigationAgent robotAgent = robot.GetComponent<SocialNavigationAgent>();
            if (robotAgent != null)
            {
                robotAgent.navAgent.SetDestination(collab.task.taskLocation);
            }
        }
    }

    void UpdateCollaborationProgress(ActiveCollaboration collab)
    {
        // Update participants on current progress
        foreach (GameObject human in collab.humanParticipants)
        {
            // Update human interface with progress
        }

        foreach (GameObject robot in collab.robotParticipants)
        {
            // Update robot with current task progress
            // This could involve sending progress updates via ROS or other communication
        }
    }

    void CompleteCollaboration(ActiveCollaboration collab)
    {
        Debug.Log($"Collaboration completed: {collab.task.taskName}");

        // Mark task as available again
        if (collab.task != null)
        {
            collab.task.isAvailable = true;
        }

        // Celebrate completion with participants
        foreach (GameObject human in collab.humanParticipants)
        {
            // Trigger celebration animation or sound
        }

        foreach (GameObject robot in collab.robotParticipants)
        {
            // Trigger completion behavior
            SocialNavigationAgent robotAgent = robot.GetComponent<SocialNavigationAgent>();
            if (robotAgent != null)
            {
                // Return to neutral state or look for new tasks
                robotAgent.navAgent.SetDestination(Vector3.zero); // Or go to neutral position
            }
        }
    }
}
```

## Communication and Feedback Systems

### Multi-Modal Communication Interface

```csharp
using UnityEngine;
using System.Collections.Generic;

public class MultiModalCommunicationInterface : MonoBehaviour
{
    [Header("Communication Modalities")]
    public bool enableSpeechSynthesis = true;
    public bool enableVisualFeedback = true;
    public bool enableGestureCommunication = true;
    public bool enableHapticFeedback = false; // If supported by hardware

    [Header("Speech Configuration")]
    public string[] greetingPhrases = { "Hello!", "Nice to meet you.", "How can I help?" };
    public string[] farewellPhrases = { "Goodbye!", "See you later.", "Thank you." };
    public float speechVolume = 1.0f;
    public float speechRate = 1.0f;

    [Header("Visual Feedback")]
    public Light statusLight;
    public Renderer[] indicatorRenderers;
    public Color idleColor = Color.blue;
    public Color busyColor = Color.yellow;
    public Color errorColor = Color.red;
    public Color interactionColor = Color.green;

    [Header("Gesture System")]
    public Animator robotAnimator;
    public List<GestureDefinition> availableGestures = new List<GestureDefinition>();

    [System.Serializable]
    public class GestureDefinition
    {
        public string gestureName;
        public string animationTrigger;
        public float duration;
        public bool isSociallyAppropriate;
        public float energyCost; // Relative energy consumption
    }

    private Dictionary<string, float> gestureCooldowns = new Dictionary<string, float>();
    private float lastSpeechTime;
    private float speechCooldown = 2.0f; // Minimum time between speech events

    void Start()
    {
        SetupCommunicationSystems();
        InitializeGestureSystem();
    }

    void SetupCommunicationSystems()
    {
        // Setup speech synthesis (in a real system, this would connect to text-to-speech)
        SetupSpeechSystem();

        // Setup visual feedback system
        SetupVisualFeedback();

        // Setup gesture system
        SetupGestureSystem();
    }

    void SetupSpeechSystem()
    {
        // In a real implementation, this would initialize text-to-speech system
        // For simulation, we'll just log speech events
        Debug.Log("Speech system initialized");
    }

    void SetupVisualFeedback()
    {
        if (statusLight != null)
        {
            statusLight.color = idleColor;
        }

        if (indicatorRenderers != null)
        {
            foreach (Renderer indicator in indicatorRenderers)
            {
                if (indicator != null)
                {
                    indicator.material.color = idleColor;
                }
            }
        }
    }

    void SetupGestureSystem()
    {
        if (robotAnimator == null)
        {
            robotAnimator = GetComponent<Animator>();
        }

        // Initialize gesture cooldowns
        foreach (GestureDefinition gesture in availableGestures)
        {
            gestureCooldowns[gesture.gestureName] = 0f;
        }

        // Create some default gestures if none exist
        if (availableGestures.Count == 0)
        {
            CreateDefaultGestures();
        }
    }

    void CreateDefaultGestures()
    {
        // Greeting gesture
        availableGestures.Add(new GestureDefinition
        {
            gestureName = "Wave",
            animationTrigger = "Wave",
            duration = 2.0f,
            isSociallyAppropriate = true,
            energyCost = 0.1f
        });

        // Nod gesture
        availableGestures.Add(new GestureDefinition
        {
            gestureName = "Nod",
            animationTrigger = "Nod",
            duration = 1.5f,
            isSociallyAppropriate = true,
            energyCost = 0.05f
        });

        // Point gesture
        availableGestures.Add(new GestureDefinition
        {
            gestureName = "Point",
            animationTrigger = "Point",
            duration = 2.5f,
            isSociallyAppropriate = true,
            energyCost = 0.15f
        });
    }

    void Update()
    {
        UpdateGestureCooldowns();
        UpdateVisualStatus();
    }

    void UpdateGestureCooldowns()
    {
        List<string> gesturesToUpdate = new List<string>(gestureCooldowns.Keys);
        foreach (string gestureName in gesturesToUpdate)
        {
            gestureCooldowns[gestureName] = Mathf.Max(0, gestureCooldowns[gestureName] - Time.deltaTime);
        }
    }

    void UpdateVisualStatus()
    {
        // Update visual indicators based on current state
        // This would typically connect to a state machine or behavior system
        Color currentColor = GetCurrentStatusColor();

        if (statusLight != null)
        {
            statusLight.color = currentColor;
        }

        if (indicatorRenderers != null)
        {
            foreach (Renderer indicator in indicatorRenderers)
            {
                if (indicator != null)
                {
                    indicator.material.color = currentColor;
                }
            }
        }
    }

    Color GetCurrentStatusColor()
    {
        // In a real implementation, this would check the robot's current state
        // For this example, we'll return a color based on activity
        if (Time.time - lastSpeechTime < 5.0f)
        {
            return interactionColor; // Recently spoke
        }
        else if (IsPerformingGesture())
        {
            return interactionColor; // Currently gesturing
        }
        else
        {
            return idleColor; // Idle state
        }
    }

    bool IsPerformingGesture()
    {
        if (robotAnimator != null)
        {
            // Check if any gesture animation is currently playing
            foreach (GestureDefinition gesture in availableGestures)
            {
                if (robotAnimator.GetCurrentAnimatorStateInfo(0).IsName(gesture.animationTrigger))
                {
                    return true;
                }
            }
        }
        return false;
    }

    public void Speak(string message, bool force = false)
    {
        if (enableSpeechSynthesis && (force || Time.time - lastSpeechTime > speechCooldown))
        {
            // In a real system, this would trigger text-to-speech
            Debug.Log($"Robot speaks: \"{message}\"");
            lastSpeechTime = Time.time;

            // Change visual status to indicate speaking
            ChangeVisualStatus(interactionColor);
        }
    }

    public void Greet()
    {
        if (greetingPhrases.Length > 0)
        {
            string greeting = greetingPhrases[Random.Range(0, greetingPhrases.Length)];
            Speak(greeting);
        }

        // Perform greeting gesture
        PerformGesture("Wave");
    }

    public void Farewell()
    {
        if (farewellPhrases.Length > 0)
        {
            string farewell = farewellPhrases[Random.Range(0, farewellPhrases.Length)];
            Speak(farewell);
        }

        // Change status to indicate end of interaction
        ChangeVisualStatus(idleColor);
    }

    public bool PerformGesture(string gestureName)
    {
        GestureDefinition gesture = availableGestures.Find(g => g.gestureName == gestureName);

        if (gesture != null)
        {
            if (gestureCooldowns.ContainsKey(gestureName) && gestureCooldowns[gestureName] <= 0)
            {
                if (robotAnimator != null)
                {
                    robotAnimator.SetTrigger(gesture.animationTrigger);

                    // Set cooldown
                    gestureCooldowns[gestureName] = gesture.duration;

                    Debug.Log($"Performed gesture: {gestureName}");
                    return true;
                }
            }
            else
            {
                Debug.Log($"Gesture '{gestureName}' is on cooldown");
            }
        }
        else
        {
            Debug.LogWarning($"Gesture '{gestureName}' not found");
        }

        return false;
    }

    void ChangeVisualStatus(Color newColor)
    {
        // Change visual indicators to new color
        if (statusLight != null)
        {
            statusLight.color = newColor;
        }

        if (indicatorRenderers != null)
        {
            foreach (Renderer indicator in indicatorRenderers)
            {
                if (indicator != null)
                {
                    indicator.material.color = newColor;
                }
            }
        }
    }

    public void SignalAttention()
    {
        // Flash visual indicators to attract attention
        StartCoroutine(FlashIndicators(interactionColor, 0.5f));
    }

    System.Collections.IEnumerator FlashIndicators(Color flashColor, float duration)
    {
        float endTime = Time.time + duration;
        Color originalColor = idleColor;

        while (Time.time < endTime)
        {
            ChangeVisualStatus(flashColor);
            yield return new WaitForSeconds(0.1f);
            ChangeVisualStatus(originalColor);
            yield return new WaitForSeconds(0.1f);
        }

        ChangeVisualStatus(originalColor);
    }

    public void ExpressEmotion(string emotion)
    {
        // Express emotion through visual, audio, and gesture cues
        switch (emotion.ToLower())
        {
            case "happy":
                ChangeVisualStatus(Color.cyan);
                PerformGesture("Nod");
                break;
            case "confused":
                ChangeVisualStatus(Color.magenta);
                Speak("I'm not sure I understand.");
                break;
            case "waiting":
                ChangeVisualStatus(Color.yellow);
                break;
            case "ready":
                ChangeVisualStatus(Color.green);
                break;
        }
    }
}
```

## Safety Considerations in Human-Robot Interaction

### Safety Management System

```csharp
using UnityEngine;
using System.Collections.Generic;

public class HRISafetyManager : MonoBehaviour
{
    [Header("Safety Configuration")]
    public float minimumSafeDistance = 0.5f;      // Minimum distance to humans
    public float collisionAvoidanceDistance = 1.0f; // Distance for active avoidance
    public float emergencyStopDistance = 0.3f;     // Distance for immediate stop
    public float maximumSpeedInHumanPresence = 0.5f; // Speed limit near humans

    [Header("Emergency Protocols")]
    public float emergencyStopDuration = 3.0f;
    public bool enableEmergencyProtocols = true;
    public float safetyCheckInterval = 0.1f;       // Check safety every 100ms

    [Header("Alert Systems")]
    public Light[] warningLights;
    public AudioSource[] alertSounds;
    public bool enableVisualAlerts = true;
    public bool enableAudioAlerts = true;

    private float lastSafetyCheck;
    private bool isEmergencyActive = false;
    private float emergencyEndTime = 0f;
    private NavMeshAgent agent;
    private List<Transform> nearbyHumans = new List<Transform>();

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        lastSafetyCheck = Time.time;
        SetupSafetySystems();
    }

    void SetupSafetySystems()
    {
        // Initialize warning lights
        if (warningLights != null)
        {
            foreach (Light light in warningLights)
            {
                if (light != null)
                {
                    light.enabled = false;
                    light.color = Color.red;
                    light.intensity = 2.0f;
                }
            }
        }

        // Initialize alert sounds
        if (alertSounds != null)
        {
            foreach (AudioSource audio in alertSounds)
            {
                if (audio != null)
                {
                    audio.volume = 0.7f;
                }
            }
        }
    }

    void Update()
    {
        if (Time.time - lastSafetyCheck >= safetyCheckInterval)
        {
            PerformSafetyCheck();
            lastSafetyCheck = Time.time;
        }

        HandleEmergencyState();
    }

    void PerformSafetyCheck()
    {
        if (isEmergencyActive)
            return;

        // Detect nearby humans
        nearbyHumans.Clear();
        Collider[] humans = Physics.OverlapSphere(transform.position, collisionAvoidanceDistance, LayerMask.GetMask("Human"));

        foreach (Collider human in humans)
        {
            nearbyHumans.Add(human.transform);
        }

        // Check safety conditions
        CheckProximitySafety();
        CheckMovementSafety();
    }

    void CheckProximitySafety()
    {
        foreach (Transform human in nearbyHumans)
        {
            float distance = Vector3.Distance(transform.position, human.position);

            if (distance < emergencyStopDistance)
            {
                // Immediate danger - activate emergency protocols
                ActivateEmergencyStop();
                return;
            }
            else if (distance < minimumSafeDistance)
            {
                // Too close - slow down and warn
                SlowDownForSafety();
                ActivateWarning();
            }
            else if (distance < collisionAvoidanceDistance)
            {
                // Near boundary - prepare for avoidance
                PrepareForAvoidance();
            }
        }
    }

    void CheckMovementSafety()
    {
        if (agent != null && agent.hasPath && nearbyHumans.Count > 0)
        {
            // Check if current path is safe
            if (!IsPathSafe(agent.path))
            {
                // Recalculate path to avoid humans
                RecalculateSafePath();
            }
        }
    }

    bool IsPathSafe(NavMeshPath path)
    {
        if (path.corners.Length < 2)
            return true;

        for (int i = 0; i < path.corners.Length - 1; i++)
        {
            Vector3 start = path.corners[i];
            Vector3 end = path.corners[i + 1];
            Vector3 direction = end - start;
            float distance = direction.magnitude;

            if (distance > 0.1f) // Avoid zero-length segments
            {
                direction.Normalize();

                // Check multiple points along the path segment
                int samples = Mathf.CeilToInt(distance / 0.5f); // Check every 0.5m
                for (int j = 0; j <= samples; j++)
                {
                    float t = (float)j / samples;
                    Vector3 samplePoint = Vector3.Lerp(start, end, t);

                    // Check for nearby humans at this point
                    foreach (Transform human in nearbyHumans)
                    {
                        if (Vector3.Distance(samplePoint, human.position) < minimumSafeDistance)
                        {
                            return false; // Path is not safe
                        }
                    }
                }
            }
        }

        return true; // Path appears safe
    }

    void RecalculateSafePath()
    {
        if (agent != null && agent.destination != Vector3.zero)
        {
            // Find alternative destination that maintains safe distance
            Vector3 safeDestination = FindSafeAlternativeDestination(agent.destination);
            agent.SetDestination(safeDestination);
        }
    }

    Vector3 FindSafeAlternativeDestination(Vector3 originalDestination)
    {
        // Try to find a destination that maintains safe distance from humans
        Vector3 directionToDestination = (originalDestination - transform.position).normalized;

        // Check if direct path is blocked
        foreach (Transform human in nearbyHumans)
        {
            Vector3 directionToHuman = (human.position - transform.position).normalized;
            float angleToHuman = Vector3.Angle(directionToDestination, directionToHuman);

            if (angleToHuman < 45f) // Human is roughly in the path direction
            {
                // Find alternative path around the human
                Vector3 perpendicular = Vector3.Cross(directionToDestination, Vector3.up).normalized;
                Vector3 alternativeDirection = directionToDestination + perpendicular * 0.5f; // Bias to one side

                Vector3 alternativeDest = transform.position + alternativeDirection.normalized * 2f; // 2m ahead

                // Ensure alternative destination is on NavMesh
                NavMeshHit hit;
                if (NavMesh.SamplePosition(alternativeDest, out hit, 3.0f, NavMesh.AllAreas))
                {
                    return hit.position;
                }
            }
        }

        // If no obstacle, return original destination
        return originalDestination;
    }

    void SlowDownForSafety()
    {
        if (agent != null)
        {
            agent.speed = Mathf.Min(agent.speed * 0.5f, maximumSpeedInHumanPresence);
        }
    }

    void PrepareForAvoidance()
    {
        if (agent != null)
        {
            agent.speed = Mathf.Min(agent.speed * 0.8f, maximumSpeedInHumanPresence);
        }
    }

    void ActivateEmergencyStop()
    {
        if (enableEmergencyProtocols)
        {
            Debug.LogWarning("EMERGENCY: Human proximity detected! Activating safety protocols.");

            isEmergencyActive = true;
            emergencyEndTime = Time.time + emergencyStopDuration;

            if (agent != null)
            {
                agent.isStopped = true;
            }

            ActivateWarning();
            PlayEmergencySound();
        }
    }

    void HandleEmergencyState()
    {
        if (isEmergencyActive && Time.time >= emergencyEndTime)
        {
            DeactivateEmergency();
        }
    }

    void DeactivateEmergency()
    {
        isEmergencyActive = false;
        if (agent != null)
        {
            agent.isStopped = false;
        }

        // Turn off warning lights
        if (enableVisualAlerts && warningLights != null)
        {
            foreach (Light light in warningLights)
            {
                if (light != null)
                {
                    light.enabled = false;
                }
            }
        }

        Debug.Log("Emergency protocols deactivated. Resuming normal operation.");
    }

    void ActivateWarning()
    {
        if (enableVisualAlerts && warningLights != null)
        {
            foreach (Light light in warningLights)
            {
                if (light != null)
                {
                    light.enabled = true;
                }
            }
        }
    }

    void PlayEmergencySound()
    {
        if (enableAudioAlerts && alertSounds != null && alertSounds.Length > 0)
        {
            AudioSource emergencySound = alertSounds[0]; // Use first available sound
            if (emergencySound != null && !emergencySound.isPlaying)
            {
                emergencySound.Play();
            }
        }
    }
}
```

## Summary

Human-robot interaction in digital twin environments requires careful consideration of social norms, safety protocols, and communication systems. Effective HRI design incorporates proxemics principles, social navigation algorithms, collaborative workspace design, and comprehensive safety management. The implementation of multi-modal communication interfaces ensures that humanoid robots can interact naturally and safely with humans in shared environments.

## Next Steps

Continue to learn about implementing learning objectives and outcomes for the Unity environments chapter.

[← Previous: Rendering Techniques](./rendering-techniques) | [Next: Unity Learning Objectives →](./unity-learning-objectives)