---
sidebar_position: 3
---

# Section 5.3: Human-Robot Interaction Interfaces

## Overview

This section covers the design and implementation of human-robot interaction interfaces in Unity for digital twin applications. Students will learn to create intuitive interfaces for controlling robots, monitoring their state, and visualizing their behavior in real-time.

## Interface Design Principles

### Human-Centered Design for Robotics

Creating interfaces that are intuitive for robot operators and engineers:

1. **Clear Visual Hierarchy**: Organize information by importance
2. **Consistent Interaction Patterns**: Use familiar UI elements
3. **Real-Time Feedback**: Provide immediate response to user actions
4. **Error Prevention**: Design interfaces to prevent dangerous commands
5. **Accessibility**: Ensure interfaces are usable by people with different abilities

### Unity UI System Setup

Setting up the UI system for human-robot interaction:

```csharp
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class HRIInterfaceManager : MonoBehaviour
{
    [Header("UI Canvas")]
    public Canvas mainCanvas;
    public EventSystem eventSystem;
    
    [Header("Control Panels")]
    public GameObject controlPanel;
    public GameObject statusPanel;
    public GameObject visualizationPanel;
    
    [Header("Interaction Elements")]
    public Button moveForwardButton;
    public Button moveBackwardButton;
    public Button turnLeftButton;
    public Button turnRightButton;
    public Slider speedSlider;
    public Toggle autonomousModeToggle;
    
    [Header("Status Display")]
    public Text statusText;
    public Text positionText;
    public Text batteryText;
    public Image batteryFill;
    
    void Start()
    {
        SetupUI();
        SetupEventHandlers();
    }

    void SetupUI()
    {
        // Ensure canvas is properly configured
        if (mainCanvas == null)
        {
            mainCanvas = FindObjectOfType<Canvas>();
        }
        
        if (eventSystem == null)
        {
            eventSystem = FindObjectOfType<EventSystem>();
        }
        
        // Configure UI elements
        ConfigureControlPanel();
        ConfigureStatusPanel();
        ConfigureVisualizationPanel();
    }

    void ConfigureControlPanel()
    {
        if (controlPanel != null)
        {
            controlPanel.SetActive(true);
        }
        
        // Configure speed slider
        if (speedSlider != null)
        {
            speedSlider.minValue = 0.1f;
            speedSlider.maxValue = 2.0f;
            speedSlider.value = 1.0f;
        }
        
        // Configure autonomous mode toggle
        if (autonomousModeToggle != null)
        {
            autonomousModeToggle.isOn = false;
        }
    }

    void ConfigureStatusPanel()
    {
        if (statusPanel != null)
        {
            statusPanel.SetActive(true);
        }
        
        // Initialize status texts
        if (statusText != null)
        {
            statusText.text = "Ready";
        }
        
        if (positionText != null)
        {
            positionText.text = "Position: (0, 0, 0)";
        }
        
        if (batteryText != null)
        {
            batteryText.text = "Battery: 100%";
        }
        
        if (batteryFill != null)
        {
            batteryFill.fillAmount = 1.0f;
        }
    }

    void ConfigureVisualizationPanel()
    {
        if (visualizationPanel != null)
        {
            visualizationPanel.SetActive(true);
        }
    }

    void SetupEventHandlers()
    {
        // Setup button event handlers
        if (moveForwardButton != null)
        {
            moveForwardButton.onClick.AddListener(() => SendCommand("MOVE_FORWARD"));
        }
        
        if (moveBackwardButton != null)
        {
            moveBackwardButton.onClick.AddListener(() => SendCommand("MOVE_BACKWARD"));
        }
        
        if (turnLeftButton != null)
        {
            turnLeftButton.onClick.AddListener(() => SendCommand("TURN_LEFT"));
        }
        
        if (turnRightButton != null)
        {
            turnRightButton.onClick.AddListener(() => SendCommand("TURN_RIGHT"));
        }
        
        // Setup slider event handler
        if (speedSlider != null)
        {
            speedSlider.onValueChanged.AddListener(OnSpeedChanged);
        }
        
        // Setup toggle event handler
        if (autonomousModeToggle != null)
        {
            autonomousModeToggle.onValueChanged.AddListener(OnAutonomousModeChanged);
        }
    }

    void OnSpeedChanged(float value)
    {
        // Handle speed change
        Debug.Log($"Speed changed to: {value}");
        SendCommand("SPEED_CHANGE", value.ToString());
    }

    void OnAutonomousModeChanged(bool isOn)
    {
        // Handle autonomous mode change
        Debug.Log($"Autonomous mode: {(isOn ? "ON" : "OFF")}");
        SendCommand(isOn ? "ENABLE_AUTONOMOUS" : "DISABLE_AUTONOMOUS");
    }

    void SendCommand(string command, string parameter = "")
    {
        // In a real implementation, this would send commands to the robot
        Debug.Log($"Command sent: {command} {parameter}");
        
        // Update status
        if (statusText != null)
        {
            statusText.text = $"Command: {command}";
        }
    }
}
```

## Robot Control Interfaces

### Teleoperation Interface

Creating interfaces for direct robot control:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class TeleoperationInterface : MonoBehaviour
{
    [Header("Teleoperation Controls")]
    public Joystick movementJoystick;
    public Button emergencyStopButton;
    public Button resetButton;
    
    [Header("Camera Control")]
    public Button cameraUpButton;
    public Button cameraDownButton;
    public Button cameraLeftButton;
    public Button cameraRightButton;
    public Slider cameraZoomSlider;
    
    [Header("Robot Reference")]
    public Transform robotTransform;
    public float movementSpeed = 1.0f;
    public float rotationSpeed = 1.0f;
    public float cameraSpeed = 2.0f;
    
    private bool emergencyStopActive = false;
    
    void Start()
    {
        SetupTeleoperationControls();
    }

    void Update()
    {
        if (!emergencyStopActive)
        {
            HandleMovementControls();
            HandleCameraControls();
        }
        
        HandleEmergencyStop();
    }

    void SetupTeleoperationControls()
    {
        // Setup emergency stop button
        if (emergencyStopButton != null)
        {
            emergencyStopButton.onClick.AddListener(TriggerEmergencyStop);
        }
        
        if (resetButton != null)
        {
            resetButton.onClick.AddListener(ResetEmergencyStop);
        }
        
        // Setup camera controls
        SetupCameraControls();
    }

    void SetupCameraControls()
    {
        if (cameraUpButton != null)
        {
            cameraUpButton.onClick.AddListener(() => MoveCamera(Vector3.up));
        }
        
        if (cameraDownButton != null)
        {
            cameraDownButton.onClick.AddListener(() => MoveCamera(Vector3.down));
        }
        
        if (cameraLeftButton != null)
        {
            cameraLeftButton.onClick.AddListener(() => MoveCamera(Vector3.left));
        }
        
        if (cameraRightButton != null)
        {
            cameraRightButton.onClick.AddListener(() => MoveCamera(Vector3.right));
        }
        
        if (cameraZoomSlider != null)
        {
            cameraZoomSlider.onValueChanged.AddListener(OnZoomChanged);
        }
    }

    void HandleMovementControls()
    {
        if (movementJoystick != null)
        {
            Vector2 joystickInput = movementJoystick.Direction;
            
            // Move robot based on joystick input
            Vector3 movement = new Vector3(joystickInput.x, 0, joystickInput.y) * movementSpeed * Time.deltaTime;
            robotTransform.Translate(movement, Space.World);
            
            // Rotate robot to face movement direction
            if (joystickInput.magnitude > 0.1f)
            {
                Vector3 lookDirection = new Vector3(joystickInput.x, 0, joystickInput.y);
                Quaternion targetRotation = Quaternion.LookRotation(lookDirection, Vector3.up);
                robotTransform.rotation = Quaternion.Slerp(
                    robotTransform.rotation, 
                    targetRotation, 
                    rotationSpeed * Time.deltaTime
                );
            }
        }
    }

    void HandleCameraControls()
    {
        // Handle camera movement based on buttons
        // This would typically control a camera following the robot
    }

    void MoveCamera(Vector3 direction)
    {
        Camera.main.transform.Translate(direction * cameraSpeed * Time.deltaTime);
    }

    void OnZoomChanged(float zoomLevel)
    {
        Camera.main.fieldOfView = Mathf.Lerp(60f, 10f, zoomLevel);
    }

    void TriggerEmergencyStop()
    {
        emergencyStopActive = true;
        Debug.Log("EMERGENCY STOP ACTIVATED");
        
        // In a real system, this would send an emergency stop command to the robot
        if (emergencyStopButton != null)
        {
            emergencyStopButton.GetComponent<Image>().color = Color.red;
        }
    }

    void ResetEmergencyStop()
    {
        emergencyStopActive = false;
        Debug.Log("Emergency stop reset");
        
        if (emergencyStopButton != null)
        {
            emergencyStopButton.GetComponent<Image>().color = Color.white;
        }
    }

    void HandleEmergencyStop()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (emergencyStopActive)
            {
                ResetEmergencyStop();
            }
            else
            {
                TriggerEmergencyStop();
            }
        }
    }
}
```

### Autonomous Mode Interface

Creating interfaces for autonomous robot operation:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class AutonomousModeInterface : MonoBehaviour
{
    [Header("Autonomous Controls")]
    public Button startMissionButton;
    public Button pauseMissionButton;
    public Button stopMissionButton;
    public Button addWaypointButton;
    public Button clearWaypointsButton;
    
    [Header("Mission Planning")]
    public InputField missionNameInput;
    public Dropdown missionTypeDropdown;
    public Toggle repeatMissionToggle;
    
    [Header("Waypoint Management")]
    public Transform waypointParent;
    public GameObject waypointPrefab;
    public Text waypointCountText;
    
    [Header("Mission Status")]
    public Text missionStatusText;
    public Slider missionProgressSlider;
    public Text timeRemainingText;
    
    private bool missionActive = false;
    private bool missionPaused = false;
    private int currentWaypoint = 0;
    private int totalWaypoints = 0;
    private float missionStartTime;
    
    void Start()
    {
        SetupAutonomousControls();
        SetupMissionPlanning();
        SetupWaypointManagement();
    }

    void Update()
    {
        UpdateMissionStatus();
    }

    void SetupAutonomousControls()
    {
        if (startMissionButton != null)
        {
            startMissionButton.onClick.AddListener(StartMission);
        }
        
        if (pauseMissionButton != null)
        {
            pauseMissionButton.onClick.AddListener(TogglePauseMission);
        }
        
        if (stopMissionButton != null)
        {
            stopMissionButton.onClick.AddListener(StopMission);
        }
        
        if (addWaypointButton != null)
        {
            addWaypointButton.onClick.AddListener(AddCurrentPositionAsWaypoint);
        }
        
        if (clearWaypointsButton != null)
        {
            clearWaypointsButton.onClick.AddListener(ClearAllWaypoints);
        }
    }

    void SetupMissionPlanning()
    {
        if (missionTypeDropdown != null)
        {
            missionTypeDropdown.ClearOptions();
            missionTypeDropdown.AddOptions(new List<string> 
            { 
                "Patrol Route", 
                "Object Inspection", 
                "Area Mapping", 
                "Delivery Mission" 
            });
        }
    }

    void SetupWaypointManagement()
    {
        totalWaypoints = 0;
        UpdateWaypointCount();
    }

    void StartMission()
    {
        if (totalWaypoints == 0)
        {
            Debug.LogWarning("Cannot start mission: No waypoints defined");
            return;
        }
        
        missionActive = true;
        missionPaused = false;
        currentWaypoint = 0;
        missionStartTime = Time.time;
        
        UpdateMissionButtons();
        UpdateMissionStatusText("Mission Started");
        
        Debug.Log($"Starting mission with {totalWaypoints} waypoints");
    }

    void TogglePauseMission()
    {
        if (!missionActive) return;
        
        missionPaused = !missionPaused;
        UpdateMissionButtons();
        UpdateMissionStatusText(missionPaused ? "Mission Paused" : "Mission Resumed");
    }

    void StopMission()
    {
        missionActive = false;
        missionPaused = false;
        currentWaypoint = 0;
        
        UpdateMissionButtons();
        UpdateMissionStatusText("Mission Stopped");
        
        Debug.Log("Mission stopped by user");
    }

    void AddCurrentPositionAsWaypoint()
    {
        // In a real implementation, this would get the robot's current position
        // For this example, we'll use the camera position as a placeholder
        Vector3 waypointPos = Camera.main.transform.position;
        
        // Create waypoint visualization
        if (waypointPrefab != null && waypointParent != null)
        {
            GameObject waypointGO = Instantiate(waypointPrefab, waypointPos, Quaternion.identity);
            waypointGO.transform.SetParent(waypointParent);
            waypointGO.name = $"Waypoint_{totalWaypoints + 1}";
        }
        
        totalWaypoints++;
        UpdateWaypointCount();
        
        Debug.Log($"Added waypoint at: {waypointPos}");
    }

    void ClearAllWaypoints()
    {
        // Clear all waypoint objects
        if (waypointParent != null)
        {
            foreach (Transform child in waypointParent)
            {
                Destroy(child.gameObject);
            }
        }
        
        totalWaypoints = 0;
        currentWaypoint = 0;
        UpdateWaypointCount();
        
        Debug.Log("All waypoints cleared");
    }

    void UpdateMissionStatus()
    {
        if (missionActive && !missionPaused && missionProgressSlider != null)
        {
            // Calculate progress based on waypoints completed
            float progress = totalWaypoints > 0 ? (float)currentWaypoint / totalWaypoints : 0f;
            missionProgressSlider.value = progress;
            
            // Update time remaining (simplified calculation)
            if (timeRemainingText != null && missionStartTime > 0)
            {
                float elapsed = Time.time - missionStartTime;
                float estimatedTotal = elapsed / progress; // This is a very simplified estimate
                float remaining = estimatedTotal - elapsed;
                
                timeRemainingText.text = $"Time Remaining: {FormatTime(remaining)}";
            }
        }
    }

    void UpdateMissionButtons()
    {
        if (startMissionButton != null)
        {
            startMissionButton.interactable = !missionActive;
        }
        
        if (pauseMissionButton != null)
        {
            pauseMissionButton.interactable = missionActive;
            pauseMissionButton.GetComponentInChildren<Text>().text = 
                missionPaused ? "Resume" : "Pause";
        }
        
        if (stopMissionButton != null)
        {
            stopMissionButton.interactable = missionActive;
        }
    }

    void UpdateMissionStatusText(string status)
    {
        if (missionStatusText != null)
        {
            missionStatusText.text = status;
        }
    }

    void UpdateWaypointCount()
    {
        if (waypointCountText != null)
        {
            waypointCountText.text = $"Waypoints: {totalWaypoints}";
        }
    }

    string FormatTime(float seconds)
    {
        int minutes = Mathf.FloorToInt(seconds / 60);
        int secs = Mathf.FloorToInt(seconds % 60);
        return $"{minutes:00}:{secs:00}";
    }
}
```

## Visualization and Monitoring Interfaces

### Real-Time Robot Status Display

Creating interfaces to monitor robot status in real-time:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class RobotStatusDisplay : MonoBehaviour
{
    [Header("Robot Status Elements")]
    public Text robotNameText;
    public Text positionText;
    public Text rotationText;
    public Text velocityText;
    public Text batteryText;
    public Image batteryFill;
    public Text statusText;
    public Text modeText;
    
    [Header("Sensor Status")]
    public Text lidarStatusText;
    public Text cameraStatusText;
    public Text imuStatusText;
    public Text gpsStatusText;
    
    [Header("System Status")]
    public Text cpuUsageText;
    public Text memoryUsageText;
    public Text temperatureText;
    public Text communicationStatusText;
    
    [Header("Robot Reference")]
    public Transform robotTransform;
    private Vector3 lastPosition;
    private float lastUpdate;
    
    void Start()
    {
        lastPosition = robotTransform.position;
        lastUpdate = Time.time;
        SetupStatusDisplay();
    }

    void Update()
    {
        UpdateRobotStatus();
    }

    void SetupStatusDisplay()
    {
        if (robotNameText != null)
        {
            robotNameText.text = robotTransform.name;
        }
    }

    void UpdateRobotStatus()
    {
        // Update position
        if (positionText != null)
        {
            positionText.text = $"Position: ({robotTransform.position.x:F2}, {robotTransform.position.y:F2}, {robotTransform.position.z:F2})";
        }
        
        // Update rotation
        if (rotationText != null)
        {
            rotationText.text = $"Rotation: ({robotTransform.eulerAngles.x:F1}, {robotTransform.eulerAngles.y:F1}, {robotTransform.eulerAngles.z:F1})";
        }
        
        // Update velocity
        if (velocityText != null)
        {
            float timeDelta = Time.time - lastUpdate;
            if (timeDelta > 0)
            {
                Vector3 velocity = (robotTransform.position - lastPosition) / timeDelta;
                velocityText.text = $"Velocity: {velocity.magnitude:F2} m/s";
            }
        }
        
        // Update battery (simulated)
        if (batteryText != null && batteryFill != null)
        {
            float batteryLevel = SimulateBatteryDischarge();
            batteryText.text = $"Battery: {batteryLevel:F0}%";
            batteryFill.fillAmount = batteryLevel / 100f;
        }
        
        // Update status
        if (statusText != null)
        {
            statusText.text = "Status: Operational";
        }
        
        // Update mode
        if (modeText != null)
        {
            modeText.text = "Mode: Autonomous";
        }
        
        // Update sensor statuses
        UpdateSensorStatuses();
        
        // Update system statuses
        UpdateSystemStatuses();
        
        // Store for next velocity calculation
        lastPosition = robotTransform.position;
        lastUpdate = Time.time;
    }

    float SimulateBatteryDischarge()
    {
        // Simulate battery discharge over time
        float dischargeRate = 0.01f; // 1% per minute
        float timeElapsed = Time.time / 60f; // Convert to minutes
        float batteryLevel = 100f - (timeElapsed * dischargeRate * 100f);
        return Mathf.Clamp(batteryLevel, 0f, 100f);
    }

    void UpdateSensorStatuses()
    {
        // Simulate sensor statuses
        if (lidarStatusText != null)
        {
            lidarStatusText.text = "LiDAR: Active";
            lidarStatusText.color = Color.green;
        }
        
        if (cameraStatusText != null)
        {
            cameraStatusText.text = "Camera: Active";
            cameraStatusText.color = Color.green;
        }
        
        if (imuStatusText != null)
        {
            imuStatusText.text = "IMU: Active";
            imuStatusText.color = Color.green;
        }
        
        if (gpsStatusText != null)
        {
            gpsStatusText.text = "GPS: Active";
            gpsStatusText.color = Color.green;
        }
    }

    void UpdateSystemStatuses()
    {
        // Simulate system statuses
        if (cpuUsageText != null)
        {
            cpuUsageText.text = $"CPU: {Random.Range(10, 40)}%";
        }
        
        if (memoryUsageText != null)
        {
            memoryUsageText.text = $"Memory: {Random.Range(30, 60)}%";
        }
        
        if (temperatureText != null)
        {
            temperatureText.text = $"Temp: {Random.Range(35, 45)}°C";
        }
        
        if (communicationStatusText != null)
        {
            communicationStatusText.text = "Comm: Connected";
            communicationStatusText.color = Color.green;
        }
    }
}
```

### Sensor Data Visualization Interface

Creating interfaces to visualize sensor data:

```csharp
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class SensorDataVisualization : MonoBehaviour
{
    [Header("Sensor Data Displays")]
    public Text lidarRangeText;
    public Text imuOrientationText;
    public Text cameraImageText; // Placeholder for camera feed
    public Text gpsCoordinatesText;
    
    [Header("Visualization Elements")]
    public Image lidarVisualization;
    public Image imuVisualization;
    public RawImage cameraFeedImage;
    public Text pointCloudText;
    
    [Header("Sensor Data Storage")]
    public float[] lidarRanges;
    public int lidarRayCount = 360;
    public float lidarMaxRange = 10.0f;
    
    [Header("Visualization Settings")]
    public Color lidarRayColor = Color.red;
    public Color obstacleColor = Color.red;
    public Color freeSpaceColor = Color.green;
    
    private Texture2D lidarTexture;
    private Color[] lidarTextureColors;
    
    void Start()
    {
        InitializeSensorData();
        SetupVisualization();
    }

    void Update()
    {
        UpdateSensorVisualizations();
    }

    void InitializeSensorData()
    {
        // Initialize lidar ranges array
        lidarRanges = new float[lidarRayCount];
        for (int i = 0; i < lidarRayCount; i++)
        {
            // Simulate initial range values
            lidarRanges[i] = Random.Range(1.0f, lidarMaxRange);
        }
    }

    void SetupVisualization()
    {
        // Create texture for lidar visualization
        if (lidarVisualization != null)
        {
            lidarTexture = new Texture2D(512, 512);
            lidarTexture.filterMode = FilterMode.Point;
            lidarTextureColors = lidarTexture.GetPixels();
            
            // Initialize with background color
            for (int i = 0; i < lidarTextureColors.Length; i++)
            {
                lidarTextureColors[i] = Color.black;
            }
            
            lidarTexture.SetPixels(lidarTextureColors);
            lidarTexture.Apply();
            
            lidarVisualization.sprite = Sprite.Create(
                lidarTexture, 
                new Rect(0, 0, lidarTexture.width, lidarTexture.height), 
                new Vector2(0.5f, 0.5f)
            );
        }
    }

    void UpdateSensorVisualizations()
    {
        UpdateLidarVisualization();
        UpdateIMUVisualization();
        UpdateCameraVisualization();
        UpdateGPSVisualization();
    }

    void UpdateLidarVisualization()
    {
        if (lidarVisualization != null && lidarTexture != null)
        {
            // Clear the texture
            for (int i = 0; i < lidarTextureColors.Length; i++)
            {
                lidarTextureColors[i] = Color.black;
            }
            
            // Draw lidar rays
            int centerX = lidarTexture.width / 2;
            int centerY = lidarTexture.height / 2;
            
            for (int i = 0; i < lidarRanges.Length; i++)
            {
                float angle = (i * 360f / lidarRanges.Length) * Mathf.Deg2Rad;
                float range = lidarRanges[i];
                
                // Scale range to texture size
                float scaledRange = (range / lidarMaxRange) * (lidarTexture.width / 2 - 10);
                
                int endX = Mathf.RoundToInt(centerX + Mathf.Cos(angle) * scaledRange);
                int endY = Mathf.RoundToInt(centerY + Mathf.Sin(angle) * scaledRange);
                
                // Draw the ray
                DrawLineOnTexture(centerX, centerY, endX, endY, lidarRayColor);
                
                // Mark endpoint if it's a valid range reading
                if (range > 0.1f && range < lidarMaxRange - 0.1f)
                {
                    DrawPointOnTexture(endX, endY, obstacleColor, 2);
                }
            }
            
            lidarTexture.SetPixels(lidarTextureColors);
            lidarTexture.Apply();
        }
        
        // Update text display
        if (lidarRangeText != null)
        {
            // Show some sample ranges
            string rangeText = "Ranges: ";
            for (int i = 0; i < Mathf.Min(5, lidarRanges.Length); i++)
            {
                rangeText += $"{lidarRanges[i]:F2}m, ";
            }
            lidarRangeText.text = rangeText;
        }
    }

    void UpdateIMUVisualization()
    {
        // Simulate IMU data
        Vector3 orientation = new Vector3(
            Random.Range(-180f, 180f),
            Random.Range(-180f, 180f),
            Random.Range(-180f, 180f)
        );
        
        if (imuOrientationText != null)
        {
            imuOrientationText.text = $"Orientation: ({orientation.x:F1}°, {orientation.y:F1}°, {orientation.z:F1}°)";
        }
        
        // Update visualization image with orientation representation
        if (imuVisualization != null)
        {
            // This would show a 3D representation of the orientation
            // For now, we'll just change color based on values
            Color orientationColor = new Color(
                Mathf.Abs(orientation.x) / 180f,
                Mathf.Abs(orientation.y) / 180f,
                Mathf.Abs(orientation.z) / 180f
            );
            imuVisualization.color = orientationColor;
        }
    }

    void UpdateCameraVisualization()
    {
        // In a real implementation, this would show the actual camera feed
        if (cameraImageText != null)
        {
            cameraImageText.text = "Camera Feed Active";
        }
    }

    void UpdateGPSVisualization()
    {
        // Simulate GPS coordinates
        float latitude = 37.7749f + Random.Range(-0.001f, 0.001f);
        float longitude = -122.4194f + Random.Range(-0.001f, 0.001f);
        
        if (gpsCoordinatesText != null)
        {
            gpsCoordinatesText.text = $"GPS: {latitude:F6}, {longitude:F6}";
        }
    }

    void DrawLineOnTexture(int x0, int y0, int x1, int y1, Color color)
    {
        // Simple line drawing algorithm
        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;

        while (true)
        {
            DrawPointOnTexture(x, y, color, 1);
            
            if (x == x1 && y == y1) break;
            
            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
        }
    }

    void DrawPointOnTexture(int x, int y, Color color, int radius)
    {
        for (int dy = -radius; dy <= radius; dy++)
        {
            for (int dx = -radius; dx <= radius; dx++)
            {
                int px = x + dx;
                int py = y + dy;
                
                if (px >= 0 && px < lidarTexture.width && 
                    py >= 0 && py < lidarTexture.height)
                {
                    float distance = Mathf.Sqrt(dx * dx + dy * dy);
                    if (distance <= radius)
                    {
                        int index = py * lidarTexture.width + px;
                        lidarTextureColors[index] = color;
                    }
                }
            }
        }
    }

    // Method to update with real sensor data (would be called from ROS bridge)
    public void UpdateLidarData(float[] newRanges)
    {
        if (newRanges.Length == lidarRanges.Length)
        {
            System.Array.Copy(newRanges, lidarRanges, newRanges.Length);
        }
        else
        {
            // Handle different array sizes
            int copyLength = Mathf.Min(newRanges.Length, lidarRanges.Length);
            for (int i = 0; i < copyLength; i++)
            {
                lidarRanges[i] = newRanges[i];
            }
        }
    }
}
```

## Safety and Error Handling Interfaces

### Safety System Interface

Creating interfaces for safety monitoring and emergency handling:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class SafetySystemInterface : MonoBehaviour
{
    [Header("Safety Status")]
    public Text safetyStatusText;
    public Image safetyStatusIndicator;
    public Text safetyMessageText;
    
    [Header("Emergency Controls")]
    public Button emergencyStopButton;
    public Button resetSystemButton;
    public Button manualOverrideButton;
    
    [Header("Safety Zones")]
    public Toggle showSafetyZonesToggle;
    public Slider safetyZoneRadiusSlider;
    public Text safetyZoneRadiusText;
    
    [Header("System Monitoring")]
    public Text collisionRiskText;
    public Text proximityWarningText;
    public Text systemIntegrityText;
    
    private bool emergencyActive = false;
    private bool manualOverrideActive = false;
    private float safetyZoneRadius = 2.0f;
    
    void Start()
    {
        SetupSafetyInterface();
    }

    void Update()
    {
        UpdateSafetyStatus();
    }

    void SetupSafetyInterface()
    {
        // Setup emergency stop button
        if (emergencyStopButton != null)
        {
            emergencyStopButton.onClick.AddListener(ActivateEmergencyStop);
        }
        
        // Setup reset button
        if (resetSystemButton != null)
        {
            resetSystemButton.onClick.AddListener(ResetSafetySystem);
        }
        
        // Setup manual override button
        if (manualOverrideButton != null)
        {
            manualOverrideButton.onClick.AddListener(ToggleManualOverride);
        }
        
        // Setup safety zone controls
        if (showSafetyZonesToggle != null)
        {
            showSafetyZonesToggle.onValueChanged.AddListener(OnShowSafetyZonesChanged);
        }
        
        if (safetyZoneRadiusSlider != null)
        {
            safetyZoneRadiusSlider.minValue = 0.5f;
            safetyZoneRadiusSlider.maxValue = 5.0f;
            safetyZoneRadiusSlider.value = safetyZoneRadius;
            safetyZoneRadiusSlider.onValueChanged.AddListener(OnSafetyZoneRadiusChanged);
        }
        
        if (safetyZoneRadiusText != null)
        {
            safetyZoneRadiusText.text = $"Safety Zone: {safetyZoneRadius:F1}m";
        }
    }

    void UpdateSafetyStatus()
    {
        // Update safety status based on various factors
        bool systemSafe = IsSystemSafe();
        
        if (safetyStatusIndicator != null)
        {
            safetyStatusIndicator.color = systemSafe ? Color.green : Color.red;
        }
        
        if (safetyStatusText != null)
        {
            safetyStatusText.text = systemSafe ? "SAFE" : "UNSAFE";
        }
        
        // Update safety message
        if (safetyMessageText != null)
        {
            safetyMessageText.text = GetSafetyMessage();
        }
        
        // Update system monitoring texts
        UpdateSystemMonitoring();
    }

    bool IsSystemSafe()
    {
        // Check various safety conditions
        return !emergencyActive && !IsCollisionImminent() && SystemIntegrityCheck();
    }

    bool IsCollisionImminent()
    {
        // Simplified collision detection
        // In a real system, this would check sensor data
        return false;
    }

    bool SystemIntegrityCheck()
    {
        // Check if all systems are functioning properly
        return true;
    }

    string GetSafetyMessage()
    {
        if (emergencyActive)
        {
            return "EMERGENCY STOP ACTIVE - SYSTEM HALTED";
        }
        else if (IsCollisionImminent())
        {
            return "COLLISION RISK DETECTED";
        }
        else if (!SystemIntegrityCheck())
        {
            return "SYSTEM INTEGRITY COMPROMISED";
        }
        else
        {
            return "SYSTEM OPERATIONAL - ALL SAFETY CHECKS PASSED";
        }
    }

    void UpdateSystemMonitoring()
    {
        if (collisionRiskText != null)
        {
            collisionRiskText.text = $"Collision Risk: {(IsCollisionImminent() ? "HIGH" : "LOW")}";
            collisionRiskText.color = IsCollisionImminent() ? Color.red : Color.green;
        }
        
        if (proximityWarningText != null)
        {
            proximityWarningText.text = "Proximity: OK";
            proximityWarningText.color = Color.green;
        }
        
        if (systemIntegrityText != null)
        {
            systemIntegrityText.text = SystemIntegrityCheck() ? "Integrity: OK" : "Integrity: COMPROMISED";
            systemIntegrityText.color = SystemIntegrityCheck() ? Color.green : Color.red;
        }
    }

    void ActivateEmergencyStop()
    {
        emergencyActive = true;
        Debug.Log("EMERGENCY STOP ACTIVATED");
        
        // Change button appearance
        if (emergencyStopButton != null)
        {
            emergencyStopButton.GetComponent<Image>().color = Color.red;
        }
        
        // In a real system, this would send emergency stop to robot
        // and potentially engage physical safety mechanisms
    }

    void ResetSafetySystem()
    {
        emergencyActive = false;
        manualOverrideActive = false;
        Debug.Log("SAFETY SYSTEM RESET");
        
        // Reset button appearance
        if (emergencyStopButton != null)
        {
            emergencyStopButton.GetComponent<Image>().color = Color.white;
        }
        
        if (manualOverrideButton != null)
        {
            manualOverrideButton.GetComponent<Image>().color = Color.white;
        }
    }

    void ToggleManualOverride()
    {
        manualOverrideActive = !manualOverrideActive;
        Debug.Log($"MANUAL OVERRIDE {(manualOverrideActive ? "ACTIVATED" : "DEACTIVATED")}");
        
        if (manualOverrideButton != null)
        {
            manualOverrideButton.GetComponent<Image>().color = 
                manualOverrideActive ? Color.yellow : Color.white;
        }
    }

    void OnShowSafetyZonesChanged(bool show)
    {
        // In a real implementation, this would show/hide safety zone visualizations
        Debug.Log($"Safety zones {(show ? "shown" : "hidden")}");
    }

    void OnSafetyZoneRadiusChanged(float radius)
    {
        safetyZoneRadius = radius;
        
        if (safetyZoneRadiusText != null)
        {
            safetyZoneRadiusText.text = $"Safety Zone: {safetyZoneRadius:F1}m";
        }
        
        Debug.Log($"Safety zone radius changed to {safetyZoneRadius:F1}m");
    }
}
```

## Best Practices for HRI Interfaces

1. **Intuitive Design**: Use familiar UI patterns and clear visual hierarchy
2. **Real-Time Feedback**: Provide immediate response to user actions
3. **Safety First**: Implement multiple safety layers and emergency procedures
4. **Accessibility**: Ensure interfaces are usable by people with different abilities
5. **Consistent Layout**: Maintain consistent positioning of controls
6. **Error Prevention**: Design interfaces to prevent dangerous commands
7. **Modular Components**: Create reusable interface components
8. **Performance**: Optimize interfaces for real-time operation

## Troubleshooting Common Issues

### Issue 1: Unresponsive Controls
- **Check**: Event system and canvas configuration
- **Verify**: Button click handlers
- **Adjust**: UI raycast settings

### Issue 2: Interface Performance Issues
- **Check**: UI update frequency
- **Verify**: Efficient UI update methods
- **Optimize**: Reduce unnecessary UI updates

### Issue 3: Safety System False Alarms
- **Check**: Safety threshold configurations
- **Verify**: Sensor data accuracy
- **Adjust**: Fine-tune safety parameters

## Next Steps

After learning about human-robot interaction interfaces, proceed to [Section 5.4: Visualization of Sensor Data in Unity](../chapter-5/section-5-4-sensor-visualization) to explore advanced techniques for visualizing sensor data in Unity.