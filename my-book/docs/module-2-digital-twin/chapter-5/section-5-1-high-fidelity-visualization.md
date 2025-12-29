---
sidebar_position: 1
---

# Section 5.1: High-Fidelity Visualization in Unity

## Overview

This section covers the creation of high-fidelity 3D visualizations in Unity for digital twin applications. Students will learn to set up realistic environments, implement advanced lighting, and create detailed humanoid robot models for visualization in digital twin simulations.

## Unity Scene Setup for Digital Twins

### Project Configuration

For digital twin applications, configure Unity with appropriate settings:

1. **Project Settings**:
   - Set up for 3D environment
   - Configure appropriate render pipeline (URP/HDRP)
   - Set physics timestep for simulation accuracy

2. **Scene Organization**:
   - Use appropriate scale (1 Unity unit = 1 meter)
   - Organize objects in logical hierarchy
   - Use meaningful naming conventions

### Basic Scene Setup

```csharp
using UnityEngine;

public class DigitalTwinSceneSetup : MonoBehaviour
{
    [Header("Environment Settings")]
    public float worldScale = 1.0f;  // 1 Unity unit = 1 meter
    public Material defaultMaterial;
    
    [Header("Lighting")]
    public Light mainLight;
    public Color ambientLightColor = new Color(0.2f, 0.2f, 0.2f, 1.0f);
    
    void Start()
    {
        SetupScene();
    }

    void SetupScene()
    {
        // Configure physics for accurate simulation
        Physics.defaultSolverIterations = 8;
        Physics.defaultSolverVelocityIterations = 2;
        
        // Set up lighting
        RenderSettings.ambientLight = ambientLightColor;
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        
        // Configure main light
        if (mainLight != null)
        {
            mainLight.type = LightType.Directional;
            mainLight.intensity = 1.0f;
            mainLight.shadows = LightShadows.Soft;
        }
    }
}
```

## Environment Creation

### Creating Realistic Environments

For humanoid robot simulation, environments should be detailed and realistic:

```csharp
using UnityEngine;

public class EnvironmentBuilder : MonoBehaviour
{
    [Header("Floor Settings")]
    public Material floorMaterial;
    public Vector2 floorSize = new Vector2(10f, 10f);
    
    [Header("Wall Settings")]
    public Material wallMaterial;
    public Vector3 wallHeight = new Vector3(3f, 3f, 3f);
    
    [Header("Obstacles")]
    public GameObject[] obstaclePrefabs;
    
    void Start()
    {
        BuildEnvironment();
    }

    void BuildEnvironment()
    {
        CreateFloor();
        CreateWalls();
        PlaceObstacles();
    }

    void CreateFloor()
    {
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        floor.name = "Floor";
        floor.transform.position = Vector3.zero;
        floor.transform.localScale = new Vector3(floorSize.x * 0.1f, 1, floorSize.y * 0.1f);
        floor.GetComponent<Renderer>().material = floorMaterial;
        
        // Add collision
        floor.layer = LayerMask.NameToLayer("Environment");
    }

    void CreateWalls()
    {
        // Create 4 walls around the environment
        CreateWall(new Vector3(0, wallHeight.y/2, floorSize.y/2), 
                   new Vector3(floorSize.x, 0.1f, 0.1f));
        CreateWall(new Vector3(0, wallHeight.y/2, -floorSize.y/2), 
                   new Vector3(floorSize.x, 0.1f, 0.1f));
        CreateWall(new Vector3(floorSize.x/2, wallHeight.y/2, 0), 
                   new Vector3(0.1f, 0.1f, floorSize.y));
        CreateWall(new Vector3(-floorSize.x/2, wallHeight.y/2, 0), 
                   new Vector3(0.1f, 0.1f, floorSize.y));
    }

    GameObject CreateWall(Vector3 position, Vector3 scale)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = "Wall";
        wall.transform.position = position;
        wall.transform.localScale = scale;
        wall.GetComponent<Renderer>().material = wallMaterial;
        wall.layer = LayerMask.NameToLayer("Environment");
        
        return wall;
    }

    void PlaceObstacles()
    {
        // Place random obstacles in the environment
        for (int i = 0; i < 5; i++)
        {
            if (obstaclePrefabs.Length > 0)
            {
                GameObject obstaclePrefab = obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)];
                Vector3 randomPos = new Vector3(
                    Random.Range(-floorSize.x/2 + 1, floorSize.x/2 - 1),
                    0.5f,
                    Random.Range(-floorSize.y/2 + 1, floorSize.y/2 - 1)
                );
                
                GameObject obstacle = Instantiate(obstaclePrefab, randomPos, Quaternion.identity);
                obstacle.name = $"Obstacle_{i}";
            }
        }
    }
}
```

## Humanoid Robot Model Creation

### Creating Detailed Robot Models

For high-fidelity visualization, humanoid robots need detailed models:

```csharp
using UnityEngine;

public class HumanoidRobotModel : MonoBehaviour
{
    [Header("Body Parts")]
    public Transform head;
    public Transform torso;
    public Transform leftArm;
    public Transform rightArm;
    public Transform leftLeg;
    public Transform rightLeg;
    
    [Header("Joint Configuration")]
    public ConfigurableJoint headJoint;
    public ConfigurableJoint leftShoulder;
    public ConfigurableJoint rightShoulder;
    public ConfigurableJoint leftHip;
    public ConfigurableJoint rightHip;
    
    [Header("Physical Properties")]
    public float robotMass = 60f;
    public float headMass = 5f;
    public float torsoMass = 30f;
    
    void Start()
    {
        SetupRobotModel();
    }

    void SetupRobotModel()
    {
        // Configure rigidbodies for physics simulation
        ConfigureRigidbody(torso.GetComponent<Rigidbody>(), torsoMass);
        ConfigureRigidbody(head.GetComponent<Rigidbody>(), headMass);
        
        // Configure joints with appropriate constraints
        ConfigureJoints();
    }

    void ConfigureRigidbody(Rigidbody rb, float mass)
    {
        if (rb != null)
        {
            rb.mass = mass;
            rb.drag = 0.1f;
            rb.angularDrag = 0.1f;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
        }
    }

    void ConfigureJoints()
    {
        // Configure head joint (neck)
        if (headJoint != null)
        {
            headJoint.xMotion = ConfigurableJointMotion.Locked;
            headJoint.yMotion = ConfigurableJointMotion.Locked;
            headJoint.zMotion = ConfigurableJointMotion.Locked;
            headJoint.angularXMotion = ConfigurableJointMotion.Limited;
            headJoint.angularYMotion = ConfigurableJointMotion.Limited;
            headJoint.angularZMotion = ConfigurableJointMotion.Limited;
            
            SoftJointLimit limit = new SoftJointLimit();
            limit.limit = 30f; // 30 degrees
            headJoint.angularYLimit = limit;
            headJoint.angularZLimit = limit;
        }
        
        // Similar configuration for other joints...
    }
}
```

## Advanced Visualization Techniques

### Realistic Materials and Textures

Creating realistic materials for digital twin visualization:

```csharp
using UnityEngine;

public class MaterialManager : MonoBehaviour
{
    [Header("Robot Materials")]
    public Material metalMaterial;
    public Material rubberMaterial;
    public Material plasticMaterial;
    
    [Header("Environment Materials")]
    public Material floorMaterial;
    public Material wallMaterial;
    
    void Start()
    {
        SetupMaterials();
    }

    void SetupMaterials()
    {
        // Configure metal material for robot parts
        if (metalMaterial != null)
        {
            metalMaterial.SetColor("_Color", new Color(0.7f, 0.7f, 0.8f));
            metalMaterial.SetFloat("_Metallic", 0.8f);
            metalMaterial.SetFloat("_Smoothness", 0.6f);
        }
        
        // Configure rubber material for feet
        if (rubberMaterial != null)
        {
            rubberMaterial.SetColor("_Color", new Color(0.2f, 0.2f, 0.2f));
            metalMaterial.SetFloat("_Metallic", 0.0f);
            metalMaterial.SetFloat("_Smoothness", 0.2f);
        }
        
        // Configure floor material
        if (floorMaterial != null)
        {
            floorMaterial.SetColor("_Color", new Color(0.5f, 0.5f, 0.5f));
            floorMaterial.SetFloat("_Metallic", 0.1f);
            floorMaterial.SetFloat("_Smoothness", 0.3f);
        }
    }
}
```

### Lighting and Shadows

Implementing realistic lighting for the digital twin environment:

```csharp
using UnityEngine;

public class LightingManager : MonoBehaviour
{
    [Header("Main Light")]
    public Light mainDirectionalLight;
    public Color lightColor = Color.white;
    public float lightIntensity = 1.0f;
    
    [Header("Additional Lights")]
    public Light[] additionalLights;
    
    [Header("Ambient Settings")]
    public Color ambientColor = new Color(0.2f, 0.2f, 0.2f, 1.0f);
    
    void Start()
    {
        SetupLighting();
    }

    void SetupLighting()
    {
        // Configure main directional light
        if (mainDirectionalLight != null)
        {
            mainDirectionalLight.type = LightType.Directional;
            mainDirectionalLight.color = lightColor;
            mainDirectionalLight.intensity = lightIntensity;
            mainDirectionalLight.shadows = LightShadows.Soft;
            mainDirectionalLight.shadowStrength = 0.8f;
            mainDirectionalLight.shadowResolution = ShadowResolution.High;
        }
        
        // Configure ambient lighting
        RenderSettings.ambientLight = ambientColor;
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
        
        // Configure additional lights
        foreach (Light light in additionalLights)
        {
            if (light != null)
            {
                light.enabled = true;
                light.shadows = LightShadows.Soft;
            }
        }
    }
}
```

## Camera Systems for Visualization

### Multiple Camera Setup

For comprehensive visualization of the digital twin:

```csharp
using UnityEngine;

public class CameraManager : MonoBehaviour
{
    [Header("Camera Types")]
    public Camera mainCamera;
    public Camera followCamera;
    public Camera topDownCamera;
    public Camera robotViewCamera;
    
    [Header("Camera Settings")]
    public float followDistance = 5f;
    public float followHeight = 3f;
    public float rotationSpeed = 2f;
    
    private Transform target; // Robot to follow
    
    void Start()
    {
        SetupCameras();
    }

    void SetupCameras()
    {
        // Configure main camera
        if (mainCamera != null)
        {
            mainCamera.fieldOfView = 60f;
            mainCamera.nearClipPlane = 0.1f;
            mainCamera.farClipPlane = 1000f;
        }
        
        // Configure follow camera
        if (followCamera != null)
        {
            followCamera.enabled = false; // Will be activated as needed
        }
        
        // Configure top-down camera
        if (topDownCamera != null)
        {
            topDownCamera.orthographic = true;
            topDownCamera.orthographicSize = 10f;
            topDownCamera.transform.position = new Vector3(0, 20, 0);
            topDownCamera.transform.rotation = Quaternion.Euler(90, 0, 0);
        }
    }

    void Update()
    {
        // Update follow camera position
        UpdateFollowCamera();
        
        // Handle camera switching
        HandleCameraSwitching();
    }

    void UpdateFollowCamera()
    {
        if (followCamera != null && target != null)
        {
            Vector3 targetPos = target.position - followCamera.transform.forward * followDistance + Vector3.up * followHeight;
            followCamera.transform.position = Vector3.Lerp(followCamera.transform.position, targetPos, Time.deltaTime * rotationSpeed);
            
            followCamera.transform.LookAt(target);
        }
    }

    void HandleCameraSwitching()
    {
        if (Input.GetKeyDown(KeyCode.C))
        {
            // Cycle through cameras
            DisableAllCameras();
            
            if (mainCamera != null) mainCamera.enabled = true;
        }
        else if (Input.GetKeyDown(KeyCode.F))
        {
            DisableAllCameras();
            
            if (followCamera != null) followCamera.enabled = true;
        }
        else if (Input.GetKeyDown(KeyCode.T))
        {
            DisableAllCameras();
            
            if (topDownCamera != null) topDownCamera.enabled = true;
        }
    }

    void DisableAllCameras()
    {
        if (mainCamera != null) mainCamera.enabled = false;
        if (followCamera != null) followCamera.enabled = false;
        if (topDownCamera != null) topDownCamera.enabled = false;
        if (robotViewCamera != null) robotViewCamera.enabled = false;
    }

    public void SetFollowTarget(Transform newTarget)
    {
        target = newTarget;
    }
}
```

## Performance Optimization

### Level of Detail (LOD) System

For maintaining performance with detailed models:

```csharp
using UnityEngine;

public class RobotLODSystem : MonoBehaviour
{
    [Header("LOD Settings")]
    public float lodDistance1 = 10f;
    public float lodDistance2 = 20f;
    public float lodDistance3 = 50f;
    
    [Header("LOD Models")]
    public GameObject lod0; // High detail
    public GameObject lod1; // Medium detail
    public GameObject lod2; // Low detail
    public GameObject lod3; // Very low detail
    
    private Camera mainCamera;
    
    void Start()
    {
        mainCamera = Camera.main;
        SetupLOD();
    }

    void SetupLOD()
    {
        // Initially show highest detail
        SetLOD(0);
    }

    void Update()
    {
        if (mainCamera != null)
        {
            float distance = Vector3.Distance(transform.position, mainCamera.transform.position);
            UpdateLOD(distance);
        }
    }

    void UpdateLOD(float distance)
    {
        if (distance < lodDistance1)
        {
            SetLOD(0); // Highest detail
        }
        else if (distance < lodDistance2)
        {
            SetLOD(1); // Medium detail
        }
        else if (distance < lodDistance3)
        {
            SetLOD(2); // Low detail
        }
        else
        {
            SetLOD(3); // Lowest detail
        }
    }

    void SetLOD(int lodLevel)
    {
        lod0.SetActive(lodLevel == 0);
        lod1.SetActive(lodLevel == 1);
        lod2.SetActive(lodLevel == 2);
        lod3.SetActive(lodLevel == 3);
    }
}
```

## Animation and Movement Visualization

### Robot Animation Controller

Visualizing robot movements and animations:

```csharp
using UnityEngine;

public class RobotAnimationController : MonoBehaviour
{
    [Header("Animation Components")]
    public Animator robotAnimator;
    public AnimationCurve walkCycle;
    public AnimationCurve armMovement;
    
    [Header("Movement Parameters")]
    public float walkSpeed = 1.0f;
    public float turnSpeed = 1.0f;
    
    private float currentWalkSpeed;
    private float currentTurnSpeed;
    
    void Start()
    {
        SetupAnimation();
    }

    void SetupAnimation()
    {
        if (robotAnimator != null)
        {
            robotAnimator.applyRootMotion = false;
        }
    }

    void Update()
    {
        UpdateAnimations();
    }

    void UpdateAnimations()
    {
        if (robotAnimator != null)
        {
            // Update animation parameters based on movement
            robotAnimator.SetFloat("Speed", currentWalkSpeed / walkSpeed);
            robotAnimator.SetFloat("Turn", currentTurnSpeed / turnSpeed);
            
            // Update based on actual movement
            UpdateMovementParameters();
        }
    }

    void UpdateMovementParameters()
    {
        // This would be updated based on actual robot movement data
        // For simulation, we can use input or simulated movement
        currentWalkSpeed = Input.GetAxis("Vertical") * walkSpeed;
        currentTurnSpeed = Input.GetAxis("Horizontal") * turnSpeed;
    }

    public void SetWalkSpeed(float speed)
    {
        currentWalkSpeed = speed;
    }

    public void SetTurnSpeed(float speed)
    {
        currentTurnSpeed = speed;
    }

    public void PlayAnimation(string animationName)
    {
        if (robotAnimator != null)
        {
            robotAnimator.Play(animationName);
        }
    }
}
```

## Quality Settings for Digital Twins

### Optimizing Unity Quality Settings

Configuring Unity for digital twin visualization:

```csharp
using UnityEngine;

public class QualitySettingsManager : MonoBehaviour
{
    [Header("Quality Presets")]
    public int ultraQualityLevel = 5;
    public int highQualityLevel = 4;
    public int mediumQualityLevel = 3;
    public int lowQualityLevel = 2;
    
    [Header("Specific Settings")]
    public bool enableShadows = true;
    public bool enableReflections = true;
    public bool enableAntiAliasing = true;
    public int antiAliasingLevel = 2;
    
    void Start()
    {
        ApplyQualitySettings();
    }

    void ApplyQualitySettings()
    {
        // Set to high quality for digital twin applications
        QualitySettings.SetQualityLevel(ultraQualityLevel, true);
        
        // Configure specific settings
        QualitySettings.shadows = enableShadows ? ShadowQuality.All : ShadowQuality.Disable;
        QualitySettings.reflectionProbeBlending = enableReflections;
        
        if (enableAntiAliasing)
        {
            QualitySettings.antiAliasing = antiAliasingLevel;
        }
        else
        {
            QualitySettings.antiAliasing = 0;
        }
        
        // Set texture quality to high
        QualitySettings.masterTextureLimit = 0; // No limit for maximum quality
        QualitySettings.anisotropicFiltering = AnisotropicFiltering.Enable;
    }

    public void SetQualityLevel(int level)
    {
        QualitySettings.SetQualityLevel(level, true);
    }
}
```

## Best Practices for High-Fidelity Visualization

1. **Appropriate Scale**: Use 1 Unity unit = 1 meter for consistency with Gazebo
2. **Realistic Materials**: Implement physically-based materials for realism
3. **Optimized Models**: Balance detail with performance using LOD systems
4. **Consistent Lighting**: Use consistent lighting across the environment
5. **Physics Accuracy**: Configure physics settings to match simulation requirements
6. **Performance Monitoring**: Continuously monitor frame rates and optimize as needed
7. **Modular Design**: Create reusable components for different robot models
8. **Quality Settings**: Adjust quality settings based on target hardware

## Troubleshooting Common Issues

### Issue 1: Poor Performance with Detailed Models
- **Check**: Model polygon count
- **Verify**: LOD system implementation
- **Optimize**: Use lower-poly models or implement more aggressive LOD

### Issue 2: Lighting Inconsistencies
- **Check**: Light settings and materials
- **Verify**: Color spaces and gamma settings
- **Adjust**: Light intensities and color temperatures

### Issue 3: Scale Mismatches
- **Check**: Unity unit to real-world scale
- **Verify**: Consistency with Gazebo simulation
- **Adjust**: Model and environment scales

## Next Steps

After learning about high-fidelity visualization in Unity, proceed to [Section 5.2: Rendering Techniques for Digital Twins](../chapter-5/section-5-2-rendering-techniques) to explore advanced rendering techniques for digital twin applications.