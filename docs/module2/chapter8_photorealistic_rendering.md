# Chapter 8: Photorealistic Rendering with Unity and Unreal for Human-Robot Interaction

## Learning Objectives
By the end of this chapter, you will be able to:
- Determine when to use Unity/Unreal instead of Gazebo's basic renderer
- Set up ROS 2 TCP connectors with Unity 2022.3 LTS or Unreal Engine 5.4
- Import high-poly visual models while maintaining low-poly collision geometry
- Achieve 90 FPS real-time rendering with 4K video export

## 8.1 When and Why to Use Unity/Unreal Instead of Gazebo's Renderer

While Gazebo Harmonic's rendering engine is sufficient for most robotics applications, certain scenarios require photorealistic rendering that only modern game engines can provide:

- **Computer Vision Training**: Generating synthetic datasets with photorealistic quality for training AI models
- **Human-Robot Interaction**: Creating immersive visual experiences for human studies
- **Public Demonstration**: High-quality visual output for presentations and demonstrations
- **Simulation-to-Reality Transfer**: Training models in visually realistic environments for better real-world performance

Gazebo's rendering is optimized for physics simulation, while Unity/Unreal prioritize visual quality and performance.

## 8.2 Setting Up Unity 2022.3 LTS with ROS 2 TCP Connector

### Prerequisites
- Unity 2022.3 LTS (Long Term Support)
- ROS 2 Iron
- Unity Robotics Package
- Python 3.8+ for ROS 2 bridge

### Installation Steps

1. **Install Unity Hub and Unity 2022.3 LTS**

2. **Create a new 3D project** and import the ROS-TCP-Connector package:
   a. Open Unity Package Manager (Window → Package Manager)
   b. Add package from git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`

3. **Import the Athena humanoid model** with proper scaling:
   a. Create a new folder: `Assets/Models/Athena`
   b. Import the visual meshes (2-million-triangle model) into this folder

### Unity ROS-TCP-Connector Setup

```csharp
// Assets/Scripts/ROSConnection.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class ROSConnection : MonoBehaviour
{
    ROSConnection m_ROSConnection;
    
    // ROS topics for Athena control
    [SerializeField] string jointStateTopic = "/athena/joint_states";
    [SerializeField] string cameraTopic = "/athena/camera/image_raw";
    
    void Start()
    {
        m_ROSConnection = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to joint state updates
        m_ROSConnection.Subscribe<sensor_msgs.msg.JointState>("/athena/joint_states", OnJointStateReceived);
        
        // Set up camera publisher
        InvokeRepeating("PublishCameraData", 0.0f, 0.1f); // Every 100ms
    }
    
    void OnJointStateReceived(sensor_msgs.msg.JointState jointState)
    {
        // Update Athena humanoid's joint positions in Unity
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = (float)jointState.position[i];
            
            // Find the joint in the Unity model and set its rotation
            Transform jointTransform = FindJointTransform(jointName);
            if (jointTransform != null)
            {
                // Update joint rotation based on received position
                UpdateJointRotation(jointTransform, jointPosition, jointName);
            }
        }
    }
    
    Transform FindJointTransform(string jointName)
    {
        // Search for the joint transform in the Athena model
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            if (child.name == jointName)
                return child;
        }
        return null;
    }
    
    void UpdateJointRotation(Transform jointTransform, float position, string jointName)
    {
        // Convert ROS joint position to Unity rotation
        // Implementation depends on joint type (revolute, prismatic, etc.)
        
        if (jointName.Contains("hip") || jointName.Contains("knee") || jointName.Contains("ankle"))
        {
            // For leg joints, apply rotation around X-axis
            jointTransform.localRotation = Quaternion.Euler(position * Mathf.Rad2Deg, 0, 0);
        }
        else if (jointName.Contains("shoulder") || jointName.Contains("elbow"))
        {
            // For arm joints, apply rotation around appropriate axis
            jointTransform.localRotation = Quaternion.Euler(0, 0, position * Mathf.Rad2Deg);
        }
    }
    
    void PublishCameraData()
    {
        // Capture camera data and send to ROS
        Camera unityCamera = GetComponent<Camera>();
        if (unityCamera != null)
        {
            // Capture image and convert to ROS Image message
            Texture2D capturedImage = CaptureCameraImage(unityCamera);
            
            // Publish to ROS topic
            sensor_msgs.msg.Image imageMsg = new sensor_msgs.msg.Image();
            // Fill the image message with captured data
            m_ROSConnection.Publish(cameraTopic, imageMsg);
        }
    }
    
    Texture2D CaptureCameraImage(Camera camera)
    {
        // Capture the camera's view as a texture
        // Implementation for capturing camera output
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = camera.targetTexture;
        
        camera.Render();
        
        Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
        image.Apply();
        
        RenderTexture.active = currentRT;
        return image;
    }
}
```

## 8.3 Importing 2-Million-Triangle Athena Model

To maintain performance while preserving visual quality, we'll implement Level of Detail (LOD) strategies:

### Athena Visual Model Import

```csharp
// Assets/Scripts/AthenaVisualModel.cs
using UnityEngine;

public class AthenaVisualModel : MonoBehaviour
{
    [Header("LOD Configuration")]
    [Range(0.1f, 2.0f)]
    public float lodDistanceMultiplier = 1.0f;
    
    [Header("Visual Components")]
    public GameObject[] lodLevels; // Different levels of detail
    public Renderer[] lowPolyCollisionRenderers; // For collision detection
    public Renderer[] highPolyVisualRenderers;   // For visual rendering
    
    [Header("Performance Settings")]
    public int targetFramerate = 90;
    public float maxDrawDistance = 100f;
    
    void Start()
    {
        ConfigureLOD();
        OptimizeForPerformance();
    }
    
    void ConfigureLOD()
    {
        // Set up LOD groups based on distance
        LODGroup lodGroup = gameObject.AddComponent<LODGroup>();
        
        LOD[] lods = new LOD[lodLevels.Length];
        for (int i = 0; i < lodLevels.Length; i++)
        {
            float screenPercentage = 1.0f / Mathf.Pow(2, i); // Each LOD is half the screen percentage of the previous
            lods[i] = new LOD(screenPercentage, lodLevels[i].GetComponentsInChildren<Renderer>());
        }
        
        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }
    
    void OptimizeForPerformance()
    {
        // Optimize materials and shaders for performance
        foreach (Renderer renderer in highPolyVisualRenderers)
        {
            // Set render queue to ensure proper blending
            foreach (Material material in renderer.materials)
            {
                material.renderQueue = 2000; // Opaque geometry
                
                // Use optimized shaders
                if (material.shader.name.Contains("Standard"))
                {
                    // Consider switching to mobile-optimized shaders for better performance
                    // material.shader = Shader.Find("Mobile/Diffuse");
                }
            }
        }
        
        // Disable shadows on complex parts if needed for performance
        foreach (Renderer renderer in highPolyVisualRenderers)
        {
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }
    }
}
```

## 8.4 Setting Up Unreal Engine 5.4 with ROS 2 Plugin

While Unity integration is more straightforward, Unreal Engine 5 offers advanced rendering with Nanite and Lumen. Here's how to set it up:

### Prerequisites
- Unreal Engine 5.4
- ros2-ue plugin
- Visual Studio 2019 or later

### Setup Steps

1. **Install the ros2-ue plugin** from the Unreal Engine Marketplace or GitHub

2. **Create a new project** with the ROS 2 template

3. **Import Athena model** with appropriate settings for Unreal Engine

### Athena Control in Unreal Engine

```cpp
// AthenaCharacter.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "ROSIntegration/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Topic.h"
#include "sensor_msgs/msg_joint_state.h"
#include "AthenaCharacter.generated.h"

UCLASS()
class PHYSICALAI_API AAthenaCharacter : public ACharacter
{
    GENERATED_BODY()

public:
    // Sets default values for this character's properties
    AAthenaCharacter();

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

public:	
    // Called every frame
    virtual void Tick(float DeltaTime) override;

    // Called to bind functionality to input
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
    
    // ROS-related functions
    void InitializeROS();
    void OnJointStateReceived(const sensor_msgs::msg::JointState& JointState);
    
private:
    UPROPERTY(VisibleAnywhere)
    URosTopic* JointStateTopic;
    
    UPROPERTY(VisibleAnywhere)
    URosBridge* ROSBridge;
    
    UPROPERTY(EditDefaultsOnly)
    FName ROSBridgeName = TEXT("ROS_BRIDGE");
    
    // Reference to the skeletal mesh component
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Mesh)
    USkeletalMeshComponent* AthenaSkeletalMesh;
};
```

```cpp
// AthenaCharacter.cpp
#include "AthenaCharacter.h"

AAthenaCharacter::AAthenaCharacter()
{
    // Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;

    // Set up skeletal mesh component
    AthenaSkeletalMesh = GetMesh();
}

void AAthenaCharacter::BeginPlay()
{
    Super::BeginPlay();

    // Initialize ROS connection
    InitializeROS();
}

void AAthenaCharacter::InitializeROS()
{
    UROSIntegrationGameInstance* ROSInstance = Cast<UROSIntegrationGameInstance>(
        GetGameInstance());
    
    if (ROSInstance)
    {
        ROSBridge = ROSInstance->ROSIntegrationCore;
        
        // Create topic for joint states
        JointStateTopic = NewObject<URosTopic>();
        JointStateTopic->Init(ROSBridge, TEXT("/athena/joint_states"), 
                             TEXT("sensor_msgs/msg/JointState"));
        
        // Set up callback for joint state messages
        JointStateTopic->OnMessageReceived.AddDynamic(this, &AAthenaCharacter::OnJointStateReceived);
    }
}

void AAthenaCharacter::OnJointStateReceived(const sensor_msgs::msg::JointState& JointState)
{
    // Update joint positions in the skeletal mesh
    for (int i = 0; i < JointState.name.size(); ++i)
    {
        const FString JointName = UTF8_TO_TCHAR(JointState.name[i].c_str());
        const float JointPosition = JointState.position[i];
        
        // Find the bone and set its rotation
        if (AthenaSkeletalMesh)
        {
            const int32 BoneIndex = AthenaSkeletalMesh->GetBoneIndex(*JointName);
            if (BoneIndex != INDEX_NONE)
            {
                // Update bone rotation based on joint position
                FRotator NewRotator = AthenaSkeletalMesh->GetBoneRotation(BoneIndex, EBoneSpaces::ComponentSpace);
                
                // Apply rotation based on joint type
                if (JointName.Contains("Hip") || JointName.Contains("Knee") || JointName.Contains("Ankle"))
                {
                    NewRotator.Pitch = FMath::RadiansToDegrees(JointPosition);
                }
                else if (JointName.Contains("Shoulder") || JointName.Contains("Elbow"))
                {
                    NewRotator.Roll = FMath::RadiansToDegrees(JointPosition);
                }
                
                AthenaSkeletalMesh->SetBoneRotation(BoneIndex, NewRotator, EBoneSpaces::ComponentSpace);
            }
        }
    }
}

void AAthenaCharacter::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void AAthenaCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);
}
```

## 8.5 Achieving 90 FPS with 4K Video Export

To maintain high performance while rendering complex models:

### Unity Performance Optimization Script

```csharp
// Assets/Scripts/PerformanceOptimizer.cs
using UnityEngine;
using UnityEngine.Rendering;

public class PerformanceOptimizer : MonoBehaviour
{
    [Header("Performance Settings")]
    [Range(30, 120)] public int targetFrameRate = 90;
    [Range(0.1f, 1.0f)] public float qualityScaler = 1.0f;
    
    [Header("LOD Settings")]
    public float lodDistanceMultiplier = 1.0f;
    public int maxRenderedObjects = 100;
    
    [Header("Rendering Settings")]
    public bool enableDynamicBatching = true;
    public bool enableGPUInstancing = true;
    public ShadowQuality shadowQuality = ShadowQuality.All;
    
    void Start()
    {
        OptimizeFrameRate();
        OptimizeQualitySettings();
        OptimizeRendering();
    }
    
    void OptimizeFrameRate()
    {
        Application.targetFrameRate = targetFrameRate;
        QualitySettings.vSyncCount = 0; // Disable vsync for consistent frame pacing
    }
    
    void OptimizeQualitySettings()
    {
        // Adjust quality settings based on performance needs
        QualitySettings.shadowDistance = 50f * qualityScaler;
        QualitySettings.shadowResolution = qualityScaler > 0.7f ? 
            ShadowResolution.High : ShadowResolution.Medium;
        QualitySettings.shadowProjection = ShadowProjection.StableFit;
        QualitySettings.shadowCascades = qualityScaler > 0.8f ? 4 : 2;
    }
    
    void OptimizeRendering()
    {
        // Configure rendering for performance
        GraphicsSettings.lightsUseLinearIntensity = true;
        
        // Enable batching for performance
        if (enableDynamicBatching)
            QualitySettings.maxQueuedJobs = 4; // Enable dynamic batching
        
        if (enableGPUInstancing)
            GraphicsSettings.lightsUseColorTemperature = true; // Enable for instancing
        
        // Set up occlusion culling if available
        if (GetComponent<OcclusionCulling> () != null)
            GetComponent<OcclusionCulling>().enabled = true;
    }
    
    void Update()
    {
        // Monitor performance and adjust settings dynamically if needed
        float currentFrameTime = Time.unscaledDeltaTime;
        float targetFrameTime = 1.0f / targetFrameRate;
        
        if (currentFrameTime > targetFrameTime * 1.1f)
        {
            // Frame rate dropping, reduce quality
            qualityScaler = Mathf.Max(0.5f, qualityScaler - 0.05f);
            OptimizeQualitySettings();
        }
        else if (currentFrameTime < targetFrameTime * 0.9f && qualityScaler < 1.0f)
        {
            // Performance good, can increase quality
            qualityScaler = Mathf.Min(1.0f, qualityScaler + 0.01f);
            OptimizeQualitySettings();
        }
    }
}
```

## 8.6 4K Video Export Setup

For high-quality video export from Unity:

```csharp
// Assets/Scripts/VideoExporter.cs
using UnityEngine;
using System.IO;

public class VideoExporter : MonoBehaviour
{
    [Header("Video Export Settings")]
    public int videoWidth = 3840;  // 4K resolution
    public int videoHeight = 2160; // 4K resolution
    public float frameRate = 60f;
    public string outputDirectory = "VideoOutput";
    
    [Header("Recording Controls")]
    public KeyCode startRecordKey = KeyCode.F10;
    public KeyCode stopRecordKey = KeyCode.F11;
    
    private bool isRecording = false;
    private int frameCount = 0;
    private RenderTexture renderTexture;
    
    void Start()
    {
        // Create render texture for high-resolution capture
        renderTexture = new RenderTexture(videoWidth, videoHeight, 24);
        renderTexture.Create();
    }
    
    void Update()
    {
        if (Input.GetKeyDown(startRecordKey))
        {
            StartRecording();
        }
        
        if (Input.GetKeyDown(stopRecordKey))
        {
            StopRecording();
        }
        
        if (isRecording)
        {
            CaptureFrame();
        }
    }
    
    void StartRecording()
    {
        isRecording = true;
        frameCount = 0;
        
        // Create output directory if it doesn't exist
        if (!Directory.Exists(outputDirectory))
        {
            Directory.CreateDirectory(outputDirectory);
        }
        
        Debug.Log($"Started recording to {outputDirectory}");
    }
    
    void StopRecording()
    {
        isRecording = false;
        Debug.Log($"Stopped recording. Captured {frameCount} frames.");
    }
    
    void CaptureFrame()
    {
        // Set the camera to render to our render texture
        Camera.main.targetTexture = renderTexture;
        Camera.main.Render();
        
        // Read the render texture to a Texture2D
        RenderTexture.active = renderTexture;
        Texture2D frameTexture = new Texture2D(videoWidth, videoHeight, TextureFormat.RGB24, false);
        frameTexture.ReadPixels(new Rect(0, 0, videoWidth, videoHeight), 0, 0);
        frameTexture.Apply();
        
        // Save as PNG
        byte[] bytes = frameTexture.EncodeToPNG();
        string filename = Path.Combine(outputDirectory, $"frame_{frameCount:D6}.png");
        File.WriteAllBytes(filename, bytes);
        
        // Clean up
        DestroyImmediate(frameTexture);
        Camera.main.targetTexture = null; // Reset to screen
        
        frameCount++;
    }
}
```

## "Pro Tips" Sidebar

- **Asset Optimization**: Use texture atlasing and mesh optimization tools to reduce draw calls and improve performance.
- **Real-time vs. Offline**: For real-time interaction, prioritize performance over visual fidelity; for video export, you can use higher quality settings.
- **Lighting Consistency**: Ensure lighting in Unity/Unreal matches real-world conditions for sim-to-real transfer.

## References to Official Documentation

- [Unity Robotics Package](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Unreal Engine ROS Integration](https://github.com/robcog-iai/UnrealROS2)
- [Unity Rendering Documentation](https://docs.unity3d.com/Manual/Rendering.html)

In the next chapter, we'll explore domain randomization techniques to generate large-scale synthetic datasets for AI training.