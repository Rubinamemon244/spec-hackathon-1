---
sidebar_position: 13
---

# High-Fidelity Environments with Unity

## Learning Objectives

By the end of this section, you will be able to:
- Create high-fidelity 3D environments for humanoid robot simulation in Unity
- Implement realistic lighting and materials for computer vision training
- Design human-robot interaction scenarios in virtual environments
- Understand Unity's role in digital twin systems for humanoid robotics

## Overview

Unity provides the high-fidelity rendering capabilities essential for creating realistic environments for humanoid robot digital twins. Unlike physics-focused simulators like Gazebo, Unity excels at visual realism, making it ideal for computer vision training, human-robot interaction scenarios, and perception system development. This section covers creating compelling virtual environments that accurately represent real-world scenarios where humanoid robots operate.

## Unity for Robotics Applications

### Core Capabilities

Unity offers several advantages for robotics applications:

- **Photorealistic Rendering**: Advanced lighting models and physically-based materials
- **Real-time Performance**: Optimized for interactive simulation and training
- **Cross-platform Support**: Can export to multiple platforms and devices
- **Extensive Asset Ecosystem**: Large library of 3D models and environments
- **Scripting Flexibility**: C# scripting for custom robot behaviors and interactions

### Robotics-Specific Features

Unity provides specialized tools for robotics development:

- **Unity Robotics Hub**: Centralized access to robotics packages and samples
- **ROS# Bridge**: Integration with ROS/ROS2 communication systems
- **ML-Agents**: Machine learning framework for robot training
- **Visual Studio Integration**: Professional development environment

## Environment Design Principles

### Realistic Scene Construction

When creating environments for humanoid robots in Unity, consider these principles:

- **Scale Accuracy**: Ensure proper real-world scale (human height ~1.7m, doorways ~2m)
- **Lighting Conditions**: Match real-world lighting scenarios for computer vision training
- **Material Properties**: Use physically-based rendering (PBR) materials for photorealism
- **Collision Meshes**: Include accurate collision detection geometry for physics simulation

### Human-Robot Interaction Spaces

Design environments that facilitate meaningful human-robot interaction:

- **Navigation Spaces**: Clear pathways that accommodate both humans and robots
- **Interaction Zones**: Areas designed for human-robot collaboration and communication
- **Furniture and Objects**: Realistic objects for robot manipulation and interaction
- **Accessibility Considerations**: Ensure environments are navigable for humanoid robots

## Key Unity Components for Robotics

### Lighting System

Unity's lighting system is crucial for realistic robot environments:

```csharp
using UnityEngine;

public class EnvironmentLighting : MonoBehaviour
{
    [Header("Lighting Configuration")]
    public Light mainLight;           // Directional light (sun/sky)
    public Light[] pointLights;       // Local light sources
    public float dayNightCycleSpeed = 1.0f;

    [Header("Realistic Parameters")]
    public float luxValue = 10000f;   // Outdoor lighting intensity
    public Color temperature = new Color(1f, 0.95f, 0.8f); // Warm white

    void Start()
    {
        ConfigureMainLight();
        ConfigurePointLights();
    }

    void ConfigureMainLight()
    {
        if (mainLight != null)
        {
            mainLight.type = LightType.Directional;
            mainLight.color = temperature;
            mainLight.intensity = luxValue / 10000f; // Normalize for Unity
            mainLight.shadows = LightShadows.Soft;
            mainLight.shadowStrength = 0.8f;
        }
    }

    void ConfigurePointLights()
    {
        foreach (Light pointLight in pointLights)
        {
            if (pointLight != null)
            {
                pointLight.type = LightType.Point;
                pointLight.range = 5f; // meters
                pointLight.intensity = 1f;
                pointLight.shadows = LightShadows.Hard;
            }
        }
    }

    void Update()
    {
        // Optional: Simulate day/night cycle
        SimulateDayNightCycle();
    }

    void SimulateDayNightCycle()
    {
        if (dayNightCycleSpeed > 0)
        {
            float time = Time.time * dayNightCycleSpeed;
            float rotation = time % 360f;

            if (mainLight != null)
            {
                mainLight.transform.rotation = Quaternion.Euler(rotation, -30f, 0f);
            }
        }
    }
}
```

### Physics Engine Integration

While Unity's physics is less sophisticated than dedicated robotics simulators, it still provides essential capabilities:

- **Rigidbody Components**: Physics-enabled objects with mass and drag
- **Colliders**: Collision detection geometry for all objects
- **Joints**: Connections between objects with constraints
- **Material Properties**: Friction, bounciness, and other physical properties

### Sensor Simulation

Unity supports various approaches to sensor simulation:

- **Camera Components**: RGB, depth, and stereo vision simulation
- **Raycasting**: Custom sensor implementations for specialized needs
- **Plugin Integration**: Third-party sensor simulation tools
- **Data Export**: Sensor data for external processing and analysis

## Creating Realistic Environments

### Indoor Environment Example

```csharp
using UnityEngine;

public class IndoorEnvironmentSetup : MonoBehaviour
{
    [Header("Room Configuration")]
    public float roomWidth = 10f;
    public float roomLength = 8f;
    public float roomHeight = 3f;

    [Header("Furniture Placement")]
    public GameObject[] furniturePrefabs;
    public Transform[] placementPositions;

    [Header("Robot Spawn Points")]
    public Transform[] spawnPoints;

    void Start()
    {
        CreateRoomStructure();
        PlaceFurniture();
        SetupRobotSpawns();
        ConfigureEnvironment();
    }

    void CreateRoomStructure()
    {
        // Create floor
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.name = "Floor";
        floor.transform.position = new Vector3(0, -roomHeight/2, 0);
        floor.transform.localScale = new Vector3(roomWidth, 0.1f, roomLength);
        floor.GetComponent<Renderer>().material.color = Color.gray;

        // Create walls
        CreateWall(new Vector3(0, 0, roomLength/2), new Vector3(roomWidth, roomHeight, 0.1f));
        CreateWall(new Vector3(0, 0, -roomLength/2), new Vector3(roomWidth, roomHeight, 0.1f));
        CreateWall(new Vector3(roomWidth/2, 0, 0), new Vector3(0.1f, roomHeight, roomLength));
        CreateWall(new Vector3(-roomWidth/2, 0, 0), new Vector3(0.1f, roomHeight, roomLength));

        // Create ceiling
        GameObject ceiling = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ceiling.name = "Ceiling";
        ceiling.transform.position = new Vector3(0, roomHeight/2, 0);
        ceiling.transform.localScale = new Vector3(roomWidth, 0.1f, roomLength);
        ceiling.GetComponent<Renderer>().material.color = Color.white;
    }

    GameObject CreateWall(Vector3 position, Vector3 scale)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = "Wall";
        wall.transform.position = position;
        wall.transform.localScale = scale;
        wall.GetComponent<Renderer>().material.color = Color.white;

        // Remove collider if this is a decorative wall, or keep it for physics
        Destroy(wall.GetComponent<BoxCollider>());
        BoxCollider newCollider = wall.AddComponent<BoxCollider>();

        return wall;
    }

    void PlaceFurniture()
    {
        for (int i = 0; i < furniturePrefabs.Length && i < placementPositions.Length; i++)
        {
            GameObject furniture = Instantiate(furniturePrefabs[i], placementPositions[i].position, placementPositions[i].rotation);
            furniture.name = furniturePrefabs[i].name;
        }
    }

    void SetupRobotSpawns()
    {
        foreach (Transform spawnPoint in spawnPoints)
        {
            // Mark spawn points visually (optional)
            GameObject spawnMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            spawnMarker.name = "SpawnPoint";
            spawnMarker.transform.position = spawnPoint.position;
            spawnMarker.transform.localScale = Vector3.one * 0.2f;
            spawnMarker.GetComponent<Renderer>().material.color = Color.green;
            spawnMarker.GetComponent<Collider>().enabled = false; // No collision
        }
    }

    void ConfigureEnvironment()
    {
        // Set up reflection probes for realistic lighting
        SetupReflectionProbes();

        // Configure occlusion culling for performance
        SetupOcclusionCulling();

        // Add post-processing effects if needed
        SetupPostProcessing();
    }

    void SetupReflectionProbes()
    {
        // Add reflection probes for realistic lighting on shiny surfaces
        foreach (Transform spawnPoint in spawnPoints)
        {
            GameObject probe = new GameObject("ReflectionProbe");
            probe.transform.position = spawnPoint.position + Vector3.up * 1.5f;
            ReflectionProbe reflectionProbe = probe.AddComponent<ReflectionProbe>();
            reflectionProbe.size = new Vector3(roomWidth, roomHeight, roomLength);
            reflectionProbe.mode = ReflectionProbeMode.Realtime;
        }
    }

    void SetupPostProcessing()
    {
        // Add ambient occlusion, bloom, and other effects for realism
        // This would require post-processing stack setup
    }
}
```

### Outdoor Environment Example

```csharp
using UnityEngine;

public class OutdoorEnvironmentSetup : MonoBehaviour
{
    [Header("Terrain Configuration")]
    public int terrainWidth = 200;
    public int terrainLength = 200;
    public float terrainHeight = 50f;

    [Header("Environmental Elements")]
    public GameObject[] treePrefabs;
    public GameObject[] buildingPrefabs;
    public GameObject skybox;

    [Header("Weather Simulation")]
    public float windStrength = 1.0f;
    public bool enableRain = false;

    void Start()
    {
        CreateTerrain();
        AddVegetation();
        AddBuildings();
        SetupSkyAndWeather();
        ConfigureEnvironmentalPhysics();
    }

    void CreateTerrain()
    {
        // Create terrain programmatically
        Terrain terrain = Terrain.CreateTerrainGameObject(new TerrainData()).GetComponent<Terrain>();
        terrain.name = "EnvironmentTerrain";

        // Configure terrain data
        terrain.terrainData.heightmapResolution = 513;
        terrain.terrainData.size = new Vector3(terrainWidth, terrainHeight, terrainLength);
        terrain.terrainData.w basemapResolution = 512;

        // Apply default terrain settings
        terrain.terrainData.SetHeights(0, 0, GenerateFlatHeightmap());

        // Add terrain texture
        SplatPrototype groundTexture = new SplatPrototype();
        groundTexture.texture = CreateSimpleTexture(Color.green);
        terrain.terrainData.splatPrototypes = new SplatPrototype[] { groundTexture };
    }

    float[,] GenerateFlatHeightmap()
    {
        int resolution = 513; // Must be 2^n + 1
        float[,] heights = new float[resolution, resolution];

        for (int y = 0; y < resolution; y++)
        {
            for (int x = 0; x < resolution; x++)
            {
                heights[x, y] = 0.5f; // Flat terrain at halfway point
            }
        }

        return heights;
    }

    Texture2D CreateSimpleTexture(Color color)
    {
        Texture2D texture = new Texture2D(32, 32);
        for (int y = 0; y < texture.height; y++)
        {
            for (int x = 0; x < texture.width; x++)
            {
                texture.SetPixel(x, y, color);
            }
        }
        texture.Apply();
        return texture;
    }

    void AddVegetation()
    {
        // Randomly place trees
        for (int i = 0; i < 50; i++)
        {
            Vector3 position = new Vector3(
                Random.Range(-terrainWidth/2f, terrainWidth/2f),
                0,
                Random.Range(-terrainLength/2f, terrainLength/2f)
            );

            GameObject tree = Instantiate(treePrefabs[Random.Range(0, treePrefabs.Length)], position, Quaternion.identity);
            tree.transform.localScale *= Random.Range(0.8f, 1.2f); // Random size variation
        }
    }

    void AddBuildings()
    {
        // Place buildings around the terrain
        for (int i = 0; i < 10; i++)
        {
            Vector3 position = new Vector3(
                Random.Range(-terrainWidth/3f, terrainWidth/3f),
                0,
                Random.Range(-terrainLength/3f, terrainLength/3f)
            );

            GameObject building = Instantiate(buildingPrefabs[Random.Range(0, buildingPrefabs.Length)], position, Quaternion.identity);
            building.transform.position = new Vector3(position.x, GetTerrainHeight(position), position.z);
        }
    }

    float GetTerrainHeight(Vector3 worldPos)
    {
        // Simplified terrain height lookup
        return 0f; // In a real implementation, this would query the terrain
    }

    void SetupSkyAndWeather()
    {
        // Set up skybox
        if (skybox != null)
        {
            RenderSettings.skybox = skybox.GetComponent<Renderer>().material;
        }

        // Configure lighting for outdoor environment
        SetupOutdoorLighting();
    }

    void SetupOutdoorLighting()
    {
        // Create and configure sun (directional light)
        GameObject sun = new GameObject("Sun");
        sun.AddComponent<Light>();
        Light sunLight = sun.GetComponent<Light>();
        sunLight.type = LightType.Directional;
        sunLight.color = Color.white;
        sunLight.intensity = 1.0f;
        sunLight.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        sunLight.shadows = LightShadows.Soft;
    }

    void ConfigureEnvironmentalPhysics()
    {
        // Set up wind zones, particle systems for weather, etc.
        Physics.WakeAllRigidbodies();
    }
}
```

## Human-Robot Interaction Scenarios

### Social Navigation Environment

Creating environments where robots must navigate around humans requires special considerations:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SocialNavigationEnvironment : MonoBehaviour
{
    [Header("Human Agent Configuration")]
    public GameObject humanPrefab;
    public int numberOfHumans = 10;
    public float humanSpeed = 1.4f; // Average walking speed in m/s

    [Header("Interaction Zones")]
    public Transform[] interactionPoints;
    public float interactionRadius = 3.0f;

    [Header("Navigation Constraints")]
    public float personalSpaceRadius = 0.8f;
    public float comfortDistance = 1.2f;

    private List<GameObject> humans = new List<GameObject>();
    private List<Vector3> destinations = new List<Vector3>();

    void Start()
    {
        SpawnHumans();
        SetupInteractionPoints();
        SetupNavigationConstraints();
    }

    void SpawnHumans()
    {
        for (int i = 0; i < numberOfHumans; i++)
        {
            // Create human at random position
            Vector3 spawnPos = new Vector3(
                Random.Range(-10f, 10f),
                0,
                Random.Range(-10f, 10f)
            );

            GameObject human = Instantiate(humanPrefab, spawnPos, Quaternion.identity);
            human.name = $"Human_{i:D2}";

            // Set random destination
            Vector3 dest = new Vector3(
                Random.Range(-10f, 10f),
                0,
                Random.Range(-10f, 10f)
            );
            destinations.Add(dest);

            humans.Add(human);
        }
    }

    void SetupInteractionPoints()
    {
        foreach (Transform point in interactionPoints)
        {
            // Visualize interaction points
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.transform.position = point.position;
            marker.transform.localScale = Vector3.one * interactionRadius * 2;
            marker.GetComponent<Renderer>().material.color = new Color(0, 0, 1, 0.2f); // Transparent blue
            marker.GetComponent<Collider>().enabled = false; // No collision
        }
    }

    void SetupNavigationConstraints()
    {
        // Create personal space indicators
        foreach (GameObject human in humans)
        {
            GameObject personalSpace = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            personalSpace.transform.position = human.transform.position + Vector3.up * 0.5f;
            personalSpace.transform.localScale = new Vector3(personalSpaceRadius * 2, 0.1f, personalSpaceRadius * 2);
            personalSpace.GetComponent<Renderer>().material.color = new Color(1, 0, 0, 0.3f); // Transparent red
            personalSpace.GetComponent<Collider>().enabled = false; // No collision
        }
    }

    void Update()
    {
        // Simple movement for humans (in real implementation, use NavMeshAgent)
        for (int i = 0; i < humans.Count; i++)
        {
            GameObject human = humans[i];
            Vector3 destination = destinations[i];

            // Move toward destination
            Vector3 direction = (destination - human.transform.position).normalized;
            human.transform.position += direction * humanSpeed * Time.deltaTime;

            // Simple pathfinding: if too close to destination, pick new one
            if (Vector3.Distance(human.transform.position, destination) < 1.0f)
            {
                destinations[i] = new Vector3(
                    Random.Range(-10f, 10f),
                    0,
                    Random.Range(-10f, 10f)
                );
            }
        }
    }
}
```

## Performance Optimization

### Level of Detail (LOD) for Robotics

```csharp
using UnityEngine;

public class RobotLODSystem : MonoBehaviour
{
    [Header("LOD Configuration")]
    public float[] lodDistances = { 10f, 30f, 60f };
    public Renderer[] lodRenderers;

    [Header("Robot Specific")]
    public Transform cameraTransform; // Robot's camera or main viewpoint

    void Start()
    {
        if (cameraTransform == null)
        {
            cameraTransform = Camera.main.transform;
        }
    }

    void Update()
    {
        float distance = Vector3.Distance(transform.position, cameraTransform.position);

        // Determine which LOD to show based on distance
        int lodLevel = GetLODLevel(distance);

        for (int i = 0; i < lodRenderers.Length; i++)
        {
            lodRenderers[i].enabled = (i == lodLevel);
        }
    }

    int GetLODLevel(float distance)
    {
        for (int i = 0; i < lodDistances.Length; i++)
        {
            if (distance <= lodDistances[i])
            {
                return i;
            }
        }
        return lodRenderers.Length - 1; // Use lowest detail if beyond max distance
    }
}
```

### Occlusion Culling

```csharp
using UnityEngine;

public class EnvironmentOcclusionCulling : MonoBehaviour
{
    [Header("Occlusion Configuration")]
    public float cullingDistance = 50f;
    public LayerMask cullingMask = -1;

    [Header("Robot View Settings")]
    public Transform robotCamera;

    void Start()
    {
        if (robotCamera == null)
        {
            robotCamera = Camera.main.transform;
        }
    }

    void Update()
    {
        // In practice, Unity's built-in occlusion culling system handles this
        // This is just to demonstrate the concept
        CullDistantObjects();
    }

    void CullDistantObjects()
    {
        GameObject[] allObjects = GameObject.FindGameObjectsWithTag("Environment");

        foreach (GameObject obj in allObjects)
        {
            if (Vector3.Distance(robotCamera.position, obj.transform.position) > cullingDistance)
            {
                obj.SetActive(false);
            }
            else
            {
                obj.SetActive(true);
            }
        }
    }
}
```

## Integration with Digital Twin Systems

### Unity-ROS Bridge Setup

```csharp
using UnityEngine;
using RosSharp;

public class UnityROSBridge : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosBridgeUrl = "ws://192.168.1.100:9090";
    public float connectionTimeout = 5.0f;

    [Header("Robot Data")]
    public string robotName = "humanoid_robot";
    public string jointStateTopic = "/joint_states";

    [Header("Sensor Simulation")]
    public string[] sensorTopics = { "/camera/rgb/image_raw", "/scan", "/imu/data" };

    private RosSocket rosSocket;

    void Start()
    {
        ConnectToROS();
        SetupSubscribers();
        SetupPublishers();
    }

    void ConnectToROS()
    {
        try
        {
            rosSocket = new RosSocket(new RosSharp.WebSocketNetTransport(rosBridgeUrl));
            Debug.Log($"Connected to ROS bridge at {rosBridgeUrl}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to connect to ROS bridge: {e.Message}");
        }
    }

    void SetupSubscribers()
    {
        // Subscribe to joint states to update robot visualization
        rosSocket.Subscribe<JointState>(jointStateTopic, OnJointStateReceived);
    }

    void SetupPublishers()
    {
        // Set up publishers for sensor data
        foreach (string topic in sensorTopics)
        {
            // Publishers would be set up here
        }
    }

    void OnJointStateReceived(JointState jointState)
    {
        // Update robot model based on received joint states
        UpdateRobotJoints(jointState);
    }

    void UpdateRobotJoints(JointState jointState)
    {
        // Update the visual representation of the robot
        // This would map ROS joint names to Unity transforms
    }

    void OnDestroy()
    {
        if (rosSocket != null)
        {
            rosSocket.Close();
        }
    }
}
```

## Summary

Unity provides essential high-fidelity rendering capabilities for digital twin environments. Its realistic graphics, lighting, and material systems make it ideal for computer vision training and human-robot interaction scenarios. When combined with other simulation tools, Unity enhances the visual fidelity of digital twin systems for humanoid robots, enabling more effective training and testing of perception systems.

## Next Steps

Continue to learn about rendering techniques and optimization strategies for Unity environments in robotics applications.

[← Previous: Physics Navigation](./physics-navigation) | [Next: Rendering Techniques →](./rendering-techniques)