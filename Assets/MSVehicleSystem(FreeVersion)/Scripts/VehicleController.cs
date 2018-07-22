using Random = UnityEngine.Random;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

#region wheelClass

[Serializable]
public class Wheel
{
  [Tooltip("In this variable you must associate the mesh of the wheel of this class.")]
  public Transform wheelMesh;

  [Tooltip("In this variable you must associate the collider of the wheel of this class.")]
  public WheelCollider wheelCollider;

  [Tooltip("If this variable is true, this wheel will receive engine torque.")]
  public bool wheelDrive = true;

  [Tooltip("If this variable is true, this wheel will receive handbrake force.")]
  public bool wheelHandBrake = true;

  [Range(-2.0f, 2.0f)] [Tooltip("In this variable you can set the horizontal offset of the sliding mark of this wheel.")]
  public float skidMarkShift = 0.0f;

  [Tooltip("If this variable is true, the wheel associated with this index will receive rotation defined by the flywheel.")]
  public bool wheelTurn = false;

  [HideInInspector] public Vector3 wheelWorldPosition;
  [HideInInspector] public Mesh rendSKDmarks;
  [HideInInspector] public bool generateSkidBool;
  [HideInInspector] public float wheelColliderRPM;
  [HideInInspector] public float forwardSkid;
  [HideInInspector] public float sidewaysSkid;
}

[Serializable]
public class VehicleWheels
{
  [Range(5, 15000)] [Tooltip("In this variable you can define the mass that the wheels will have. The script will leave all wheels with the same mass.")]
  public int wheelMass = 100;

  [Space(10)] [Tooltip("The front right wheel collider must be associated with this variable")]
  public Wheel rightFrontWheel;

  [Tooltip("The front left wheel collider must be associated with this variable")]
  public Wheel leftFrontWheel;

  [Tooltip("The rear right wheel collider should be associated with this variable")]
  public Wheel rightRearWheel;

  [Tooltip("The rear left wheel collider should be associated with this variable")]
  public Wheel leftRearWheel;
}

#endregion

#region vehicleAdjustment

[Serializable]
public class VehicleAdjustmentClassFree
{
  [Tooltip("If this variable is true, the vehicle will start with the engine running. But this only applies if the player starts inside this vehicle.")]
  public bool startOn = true;

  [Range(500, 2000000)] [Tooltip("In this variable you must define the mass that the vehicle will have. Common vehicles usually have a mass around 1500")]
  public int vehicleMass = 2000;

  [Tooltip("In this variable there are some variables that allow to improve the control of the vehicle.")]
  public StabilizeTurnsClassFree improveControl;

  [Tooltip("In this class you can adjust some forces that the vehicle receives, such as gravity simulation.")]
  public AerodynamicAdjustmentClassFree _aerodynamics;

  [Space(10)] [Tooltip("In this variable an empty object affiliated to the vehicle should be associated with the center position of the vehicle, perhaps displaced slightly downward, with the intention of representing the center of mass of the vehicle.")]
  public Transform centerOfMass;

  [Tooltip("The steering wheel of the vehicle")]
  public GameObject volant;

  [HideInInspector] public AnimationCurve angle_x_Velocity = new AnimationCurve(new Keyframe(0, 1), new Keyframe(500, 0.8f));
}

[Serializable]
public class StabilizeTurnsClassFree
{
  [Range(0.0f, 1.2f)] [Tooltip("How much the code will stabilize the vehicle's skidding.")]
  public float tireSlipsFactor = 0.85f;

  [Range(0.1f, 2.0f)] [Tooltip("This variable defines how much lateral force the vehicle will receive when the steering wheel is rotated. This helps the vehicle to rotate more realistically.")]
  public float helpToTurn = 0.35f;

  [Range(0.1f, 1.0f)] [Tooltip("This variable defines how fast the vehicle will straighten automatically. This occurs naturally in a vehicle when it exits a curve.")]
  public float helpToStraightenOut = 0.1f;

  [Range(0.1f, 5.0f)] [Tooltip("This variable defines how much downforce the vehicle will receive. This helps to simulate a more realistic gravity, but should be set up carefully so as not to make some surreal situations.")]
  public float downForce = 2.0f;
}

#endregion

#region cameraClass

[Serializable]
public class VehicleCamerasClassFree
{
  [Tooltip("If this variable is checked, the script will automatically place the 'IgnoreRaycast' layer on the player when needed.")]
  public bool setLayers = false;

  [Tooltip("Here you must associate all the cameras that you want to control by this script, associating each one with an index and selecting your preferences.")]
  public CameraTypeClassFree[] cameras;

  [Tooltip("Here you can configure the cameras, deciding their speed of movement, rotation, zoom, among other options.")]
  public CameraSettingsClassFree cameraSettings;
}

[Serializable]
public class CameraTypeClassFree
{
  [Tooltip("A camera must be associated with this variable. The camera that is associated here, will receive the settings of this index.")]
  public Camera _camera;

  public enum TipoRotac
  {
    LookAtThePlayer,
    FirstPerson,
    FollowPlayer,
    Orbital,
    Stop,
    StraightStop,
    OrbitalThatFollows,
    ETS_StyleCamera
  }

  [Tooltip("Here you must select the type of rotation and movement that camera will possess.")]
  public TipoRotac rotationType = TipoRotac.FollowPlayer;

  [Range(0.01f, 1.0f)] [Tooltip("Here you must adjust the volume that the camera attached to this element can perceive. In this way, each camera can perceive a different volume.")]
  public float volume = 1.0f;
}

[Serializable]
public class CameraSettingsClassFree
{
  [Range(0.01f, 0.5f)] [Tooltip("The near the camera should possess. This parameter will be adjusted automatically depending on the type of camera.")]
  public float near = 0.03f;

  [Range(0.0f, 0.5f)] [Tooltip("How much the camera shakes when the vehicle hits something.")]
  public float impactTremor = 0.1f;

  [Tooltip("Here you can configure the preferences of ETS_StyleCamera style cameras.")]
  public ETSStyleCameraCameraSettingsClassFree ETS_StyleCamera;

  [Tooltip("Here you can configure the preferences of the cameras that follow the player.")]
  public FollowPlayerCameraSettingsClassFree followPlayerCamera;

  [Tooltip("Here you can configure the preferences of the cameras in first person.")]
  public FirstPersonCameraSettingsClassFree firstPersonCamera;

  [Tooltip("Here you can configure the preferences of cameras that orbit the player.")]
  public OrbitalCameraSettingsClassFree orbitalCamera;
}

[Serializable]
public class ETSStyleCameraCameraSettingsClassFree
{
  [Range(1, 20)] [Tooltip("In this variable you can configure the sensitivity with which the script will perceive the movement of the mouse. This is applied to cameras that interact with mouse movement only.")]
  public float sensibility = 10.0f;

  [Range(0.5f, 3.0f)] [Tooltip("The distance the camera will move to the left when the mouse is also shifted to the left. This option applies only to cameras that have the 'ETS_StyleCamera' option selected.")]
  public float ETS_CameraShift = 2.0f;
}

[Serializable]
public class FirstPersonCameraSettingsClassFree
{
  [Range(1, 20)] [Tooltip("In this variable you can configure the sensitivity with which the script will perceive the movement of the mouse. This is applied to cameras that interact with mouse movement only.")]
  public float sensibility = 10.0f;

  [Range(0, 160)] [Tooltip("The highest horizontal angle that camera style 'FistPerson' camera can achieve.")]
  public float horizontalAngle = 65.0f;

  [Range(0, 85)] [Tooltip("The highest vertical angle that camera style 'FistPerson' camera can achieve.")]
  public float verticalAngle = 20.0f;
}

[Serializable]
public class FollowPlayerCameraSettingsClassFree
{
  [Range(1, 30)] [Tooltip("The speed at which the camera rotates as it follows and looks at the player.")]
  public float spinSpeed = 15.0f;

  [Range(1, 20)] [Tooltip("The speed at which the camera can follow the player.")]
  public float displacementSpeed = 0.0f;

  [Tooltip("If this variable is true, the camera that follows the player will do a custom 'LookAt'. Slower")]
  public bool customLookAt = false;
}

[Serializable]
public class OrbitalCameraSettingsClassFree
{
  [Range(0.01f, 2.0f)] [Tooltip("In this variable you can configure the sensitivity with which the script will perceive the movement of the mouse. ")]
  public float sensibility = 0.8f;

  [Range(0.01f, 2.0f)] [Tooltip("In this variable, you can configure the speed at which the orbital camera will approach or distance itself from the player when the mouse scrool is used.")]
  public float speedScrool = 1.0f;

  [Range(0.01f, 2.0f)] [Tooltip("In this variable, you can configure the speed at which the orbital camera moves up or down.")]
  public float speedYAxis = 0.5f;

  [Range(3.0f, 20.0f)] [Tooltip("In this variable, you can set the minimum distance that the orbital camera can stay from the player.")]
  public float minDistance = 5.0f;

  [Range(20.0f, 1000.0f)] [Tooltip("In this variable, you can set the maximum distance that the orbital camera can stay from the player.")]
  public float maxDistance = 500.0f;

  [Tooltip("If this variable is true, the orbital camera has the axes reversed when the Joystick is active.")]
  public bool invertRotationJoystick = true;
}

#endregion

#region vehicleTorqueClass

[Serializable]
public class TorqueAdjustmentClassFree
{
  [Range(20, 420)] [Tooltip("This variable sets the maximum speed that the vehicle can achieve. It must be configured on the KMh unit")]
  public int maxVelocityKMh = 250;

  [Range(0.5f, 2000.0f)] [Tooltip("This variable defines the torque that the motor of the vehicle will have.")]
  public float engineTorque = 3;

  [Range(2, 12)] [Tooltip("This variable defines the number of gears that the vehicle will have.")]
  public int numberOfGears = 6;

  [Range(0.5f, 2.0f)] [Tooltip("This variable defines the speed range of each gear. The higher the range, the faster the vehicle goes, however, the torque is relatively lower.")]
  public float speedOfGear = 1.5f;

  [Range(0.5f, 2.0f)] [Tooltip("In this variable, you can manually adjust the torque that each gear has. But it is not advisable to change these values.")]
  public float[] manualAdjustmentOfTorques = new float[12] {1.1f, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  [HideInInspector] public AnimationCurve[] gears = new AnimationCurve[12]
  {
    new AnimationCurve(new Keyframe(0, 1.5f), new Keyframe(10, 2.0f), new Keyframe(30, 0)),
    new AnimationCurve(new Keyframe(0, 0.2f), new Keyframe(30, 1), new Keyframe(45, 0)),
    new AnimationCurve(new Keyframe(0, 0.2f), new Keyframe(45, 1), new Keyframe(60, 0)),
    new AnimationCurve(new Keyframe(15, 0.0f), new Keyframe(60, 1), new Keyframe(75, 0)),
    new AnimationCurve(new Keyframe(30, 0.0f), new Keyframe(75, 1), new Keyframe(90, 0)),
    new AnimationCurve(new Keyframe(45, 0.0f), new Keyframe(90, 1), new Keyframe(105, 0)),
    new AnimationCurve(new Keyframe(60, 0.0f), new Keyframe(105, 1), new Keyframe(120, 0)),
    new AnimationCurve(new Keyframe(75, 0.0f), new Keyframe(120, 1), new Keyframe(135, 0)),
    new AnimationCurve(new Keyframe(90, 0.0f), new Keyframe(135, 1), new Keyframe(150, 0)),
    new AnimationCurve(new Keyframe(105, 0.0f), new Keyframe(150, 1), new Keyframe(165, 0)),
    new AnimationCurve(new Keyframe(120, 0.0f), new Keyframe(165, 1), new Keyframe(180, 0)),
    new AnimationCurve(new Keyframe(135, 0.0f), new Keyframe(180, 1), new Keyframe(195, 0)),
  };

  [HideInInspector] public int[] minVelocityGears = new int[12] {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165};
  [HideInInspector] public int[] idealVelocityGears = new int[12] {10, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180};
  [HideInInspector] public int[] maxVelocityGears = new int[12] {30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195};
}

#endregion

#region vehicleSkidMarksClass

[Serializable]
public class VehicleSkidMarksClassFree
{
  [Range(0.1f, 6.0f)] [Tooltip("This variable defines the width of the vehicle's skid trace.")]
  public float standardBrandWidth = 0.3f;

  [Range(1.0f, 10.0f)] [Tooltip("This variable sets the sensitivity of the vehicle to start generating traces of skidding. The more sensitive, the easier to generate the traces.")]
  public float sensibility = 2.0f;

  [Tooltip("This variable sets the default color of the skid marks.")]
  public Color standardColor = new Color(0.15f, 0.15f, 0.15f, 0);
}

#endregion

#region aerodynamicClass

[Serializable]
public class AerodynamicAdjustmentClassFree
{
  [Tooltip("If this variable is true, the script will simulate a force down on the vehicle, leaving jumps more realistic.")]
  public bool extraGravity = true;

  [Range(0.0f, 10.0f)] [Tooltip("This variable defines how much force will be added to the vehicle suspension to avoid rotations. This makes the vehicle more rigid and harder to knock over.")]
  public float feelingHeavy = 1.0f;

  [Range(0.00f, 1.0f)] [Tooltip("This variable defines the amount of force that will be simulated in the vehicle while it is tilted, the steeper it is, the lower the force applied. Values too high make the vehicle too tight and prevent it from slipping.")]
  public float downForceAngleFactor = 0.2f;

  [Range(0, 1.0f)] [Tooltip("This variable defines how much force will be simulated in the vehicle while on flat terrain. Values too high cause the suspension to reach the spring limit.")]
  public float verticalDownForce = 0.8f;

  [Range(0, 3)] [Tooltip("This variable defines a minimum force value that will be simulated. The value corresponds to the mass of the vehicle times the value of this variable.")]
  public int minDownForceValue = 2;
}

#endregion

#region soundsClass

[Serializable]
public class VehicleSoundsClassFree
{
  [Range(1.5f, 6.0f)] [Tooltip("This variable defines the speed of the engine sound.")]
  public float speedOfEngineSound = 3.5f;

  [Tooltip("The audio referring to the sound of the engine must be associated here.")]
  public AudioClip engineSound;

  [Space(10)] [Tooltip("The sliding sounds of the vehicle must be set here.")]
  public StandardSkiddingSoundsClassFree skiddingSound;

  [Space(10)] [Tooltip("Collision sounds should be associated with this list.")]
  public AudioClip[] collisionSounds;

  [Range(0.1f, 1.0f)] [Tooltip("In this variable it is possible to set the volume of collision sounds of the vehicle.")]
  public float volumeCollisionSounds = 0.5f;

  [Space(10)] [Tooltip("The sound related to a collision in the wheel must be associated with this variable.")]
  public AudioClip wheelImpactSound;
}

[Serializable]
public class StandardSkiddingSoundsClassFree
{
  [Tooltip("The default sound that will be emitted when the vehicle slides or skates.")]
  public AudioClip standardSound;

  [Range(0.1f, 3.0f)] [Tooltip("The default volume of the skid sound.")]
  public float standardVolume = 1.5f;
}

#endregion

[RequireComponent(typeof(Rigidbody))]
public class VehicleController : MonoBehaviour
{
  [Space(7)] [Tooltip("In this variable, empty objects must be associated with positions close to the vehicle doors.")]
  public GameObject[] doorPosition;


  [Space(7)] [Tooltip("In this class must be configured the cameras that the vehicle has.")]
  public VehicleCamerasClassFree _cameras;

  [Tooltip("In this class you can configure the vehicle torque, number of gears and their respective torques.")]
  public TorqueAdjustmentClassFree _vehicleTorque;

  [Tooltip("In this class you can adjust various settings that allow changing the way the vehicle is controlled, as well as the initial state of some variables, such as the engine and the brake.")]
  public VehicleAdjustmentClassFree _vehicleSettings;

  [Tooltip("In this class, you can adjust all wheels of the vehicle separately, each with its own settings.")]
  public VehicleWheels _wheels;

  [Tooltip("In this class, you can adjust all vehicle sounds, and the preferences of each.")]
  public VehicleSoundsClassFree _sounds;

  [Tooltip("In this class, you can adjust all preferences in relation to vehicle skid marks, such as color, width, among other options.")]
  public VehicleSkidMarksClassFree _skidMarks;

  [Tooltip("In this variable, the 'SkidMarks' shader must be associated. Otherwise, the vehicle will not generate skid marks.")]
  public Shader skidMarksShader;

  #region inputs

  public float verticalInput = 0;
  public float horizontalInput = 0;
  public bool started;
  private float mouseXInput = 0;
  private float mouseYInput = 0;
  private float mouseScrollWheelInput = 0;

  #endregion

  private bool changinGearsAuto;
  private bool theEngineIsRunning = true;
  private bool enableEngineSound;
  private bool youCanCall;
  private bool brakingAuto;
  private bool colliding;

  private bool enableSkidMarksOnStart;

  private int groundedWheels;
  private float sumRPM;
  private float mediumRPM;
  private float angle1Ref;
  private float angle2Volant;
  private float volantStartRotation;
  private float minPitchAud;
  private float leftDifferential;
  private float rightDifferential;
  private float timeAutoGear;
  private float reverseForce;
  private float engineInput;
  private float angleRefVolant;
  private float pitchAUD = 1;
  private float speedLerpSound = 1;
  private float engineSoundFactor;
  private float vehicleScale;

  private float maxAngleVolant;

  private float torqueM;
  private float rpmTempTorque;
  private float clampInputTorque;
  private float adjustTorque;

  private bool isGroundedExtraW;
  private Vector3 axisFromRotate;
  private Vector3 torqueForceAirRotation;

  private Vector2 tireSlipTireSlips;
  private Vector2 tireForceTireSlips;
  private Vector2 localRigForceTireSlips;
  private Vector2 localVelocityWheelTireSlips;
  private Vector2 localSurfaceForceDTireSlips;
  private Vector2 rawTireForceTireSlips;
  private Vector2 tempLocalVelocityVector2;
  private Vector3 tempWheelVelocityVector3;
  private Vector3 velocityLocalWheelTemp;
  private Vector2 surfaceLocalForce;
  private Vector3 surfaceLocalForceTemp;
  private Vector3 wheelSpeedLocalSurface;
  private Vector3 downForceUPTemp;
  private float normalTemp;
  private float forceFactorTempLocalSurface;
  private float downForceTireSlips;
  private float estimatedSprungMass;
  private float angularWheelVelocityTireSlips;
  private float wheelMaxBrakeSlip;
  private float minSlipYTireSlips;
  private float maxFyTireSlips;

  private float brakeInput;
  private float velxCurrentRPM;
  private float nextPitchAUD;

  private float lastRightForwardPositionY;
  private float lastLeftForwardPositionY;
  private float lastRightRearPositionY;
  private float lastLeftRearPositionY;
  private float sensImpactFR;
  private float sensImpactFL;
  private float sensImpactRR;
  private float sensImpactRL;
  private float additionalCurrentGravity;
  private float currentBrakeValue;
  private float forceEngineBrake;

  private float sidewaysSlipMaxSkid;
  private float forwardSlipMaxSkid;
  private float sensibility75kmh;
  private float sensibilityLowSpeed;
  private float maxSlipTemp;
  private bool forwardTempSKid;
  private bool forwardHandBrakeSKid;
  private bool skiddingIsTrue;
  private int wheelsInOtherGround;
  private int maxWheels;

  private float currentDownForceVehicle;

  private Rigidbody ms_Rigidbody;

  private AudioSource engineSoundAUD;
  private AudioSource beatsSoundAUD;
  private AudioSource beatsOnWheelSoundAUD;
  private AudioSource skiddingSoundAUD;

  private WheelCollider[] wheelColliderList;
  private Vector2 tireSL;
  private Vector2 tireFO;

  private Vector3 lateralForcePointTemp;
  private Vector3 forwardForceTemp;
  private Vector3 lateralForceTemp;
  private float distanceXForceTemp;

  private WheelFrictionCurve wheelFrictionCurveFW;
  private WheelFrictionCurve wheelFrictionCurveSW;

  private WheelHit tempWheelHit;

  private Quaternion tempRotStabilizers;
  private float leftFrontForce;
  private float rightFrontForce;
  private float leftRearForce;
  private float rightRearForce;
  private float roolForce1;
  private float roolForce2;

  private float gravityValueFixedUpdate;
  private float downForceValueFixedUpdate;
  private float inclinationFactorForcesDown;
  private float downForceUpdateRef;
  private float downForceTempLerp;
  private Vector3 impactVelocityFixedUpdate;
  private Vector3 contactPointFixedUpdate;

  private bool isBraking;
  private float brakeVerticalInput;
  private float handBrake_Input;
  private float totalFootBrake;
  private float totalHandBrake;
  private float absBrakeInput;
  private float absSpeedFactor;

  private bool wheelFDIsGrounded;
  private bool wheelFEIsGrounded;
  private bool wheelTDIsGrounded;
  private bool wheelTEIsGrounded;

  private readonly Dictionary<Mesh, int> currentIndexes = new Dictionary<Mesh, int>();
  private float tempAlphaSkidMarks;
  private WheelHit tempWHeelHit;
  private Vector3 skidTemp;
  private Vector3[] lastPoint;
  private List<Vector3> vertices;
  private List<Vector3> normals;
  private List<Color> colors;
  private List<Vector2> uv;
  private List<int> tris;

  private Vector3 vectorMeshPos1;
  private Vector3 vectorMeshPos2;
  private Vector3 vectorMeshPos3;
  private Vector3 vectorMeshPos4;
  private Vector3 vectorMeshPosTemp;
  private Quaternion quatMesh1;
  private Quaternion quatMesh2;
  private Quaternion quatMesh3;
  private Quaternion quatMesh4;
  private Quaternion quatMeshTemp;

  private bool changeTypeCamera;
  private float timeScaleSpeed;
  private float minDistanceOrbitalCamera;
  private float camerasMovX;
  private float camerasMovY;
  private float camerasMovZ;
  private Quaternion xQuaternionCameras;
  private Quaternion yQuaternionCameras;
  private Quaternion newRotationCameras;
  private Quaternion currentRotationCameras;
  private Vector3 newDistanceCameras;
  private Vector3 currentPositionCameras;
  private Vector3 positionCameras;
  private RaycastHit hitCameras;

  private int indexCamera = 0;
  private bool orbitalOn;
  private float rotationX = 0.0f;
  private float rotationY = 0.0f;
  private float orbitTime = 0.0f;
  private float rotationXETS = 0.0f;
  private float rotationYETS = 0.0f;
  private GameObject[] objStraightStopCameras;
  private Quaternion[] startRotationCameras;
  private GameObject[] startPositionCameras;
  private float[] xOrbit, yOrbit, distanceOrbitCamera;
  private Vector3[] startETSCamerasPosition;
  private Vector3 tempStartPositionShakeCameras;

  [HideInInspector] public float KMh;
  [HideInInspector] public int currentGear;
  [HideInInspector] public bool handBrakeTrue;
  [HideInInspector] public bool isInsideTheCar;

  private void Awake()
  {
    enableSkidMarksOnStart = true;

    DebugStartErrors();

    SetCameras();
  }

  private void DebugStartErrors()
  {
    if (!_wheels.rightFrontWheel.wheelCollider || !_wheels.leftFrontWheel.wheelCollider || !_wheels.rightRearWheel.wheelCollider || !_wheels.leftRearWheel.wheelCollider)
    {
      Debug.LogError("The vehicle must have at least the four main wheels associated with its variables, within class '_wheels'.");
      this.transform.gameObject.SetActive(false);
      return;
    }

    if (!_wheels.rightFrontWheel.wheelMesh || !_wheels.leftFrontWheel.wheelMesh || !_wheels.rightRearWheel.wheelMesh || !_wheels.leftRearWheel.wheelMesh)
    {
      Debug.LogError("The meshes of the four main wheels must be associated with their respective variables within the class '_wheels'.");
      this.transform.gameObject.SetActive(false);
      return;
    }
  }

  private void Start()
  {
    SetValues();

    if (skidMarksShader)
    {
      if (enableSkidMarksOnStart)
      {
        SetSkidMarksValues();
      }
    }
    else
    {
      enableSkidMarksOnStart = false;
    }
  }

  private void SetValues()
  {
    forceEngineBrake = 0.75f * _vehicleSettings.vehicleMass;
    vehicleScale = transform.lossyScale.y;

    vertices = new List<Vector3>(1200);
    normals = new List<Vector3>(1200);
    colors = new List<Color>(1200);
    uv = new List<Vector2>(1200);
    tris = new List<int>(3600);
    lastPoint = new Vector3[4];

    if (doorPosition.Length == 0)
    {
      doorPosition = new GameObject[1];
    }

    for (int x = 0; x < doorPosition.Length; x++)
    {
      if (!doorPosition[x])
      {
        doorPosition[x] = new GameObject("doorPos");
        doorPosition[x].transform.position = transform.position;
      }

      doorPosition[x].transform.rotation = transform.rotation;
      doorPosition[x].transform.parent = transform;
    }

    if (!isInsideTheCar)
    {
      EnableCameras(-1);
      _vehicleSettings.startOn = false;
    }
    else
    {
      EnableCameras(indexCamera);
    }

    wheelColliderList = new WheelCollider[(4)];
    wheelColliderList[0] = _wheels.rightFrontWheel.wheelCollider;
    wheelColliderList[1] = _wheels.leftFrontWheel.wheelCollider;
    wheelColliderList[2] = _wheels.rightRearWheel.wheelCollider;
    wheelColliderList[3] = _wheels.leftRearWheel.wheelCollider;

    currentDownForceVehicle = _vehicleSettings.improveControl.downForce;

    youCanCall = true;
    handBrakeTrue = false;

    ms_Rigidbody = GetComponent<Rigidbody>();
    ms_Rigidbody.useGravity = true;
    ms_Rigidbody.mass = _vehicleSettings.vehicleMass;
    ms_Rigidbody.drag = 0.0f;
    ms_Rigidbody.angularDrag = 0.05f;
    ms_Rigidbody.maxAngularVelocity = 14.0f;
    ms_Rigidbody.maxDepenetrationVelocity = 8.0f;
    additionalCurrentGravity = 4.0f * ms_Rigidbody.mass;
    ms_Rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
    ms_Rigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

    WheelCollider WheelColliders = GetComponentInChildren<WheelCollider>();

    WheelColliders.ConfigureVehicleSubsteps(1000.0f, 20, 20);

    if (_vehicleSettings.centerOfMass)
    {
      ms_Rigidbody.centerOfMass = transform.InverseTransformPoint(_vehicleSettings.centerOfMass.position);
    }
    else
    {
      ms_Rigidbody.centerOfMass = Vector3.zero;
    }

    if (_vehicleSettings.volant)
    {
      volantStartRotation = _vehicleSettings.volant.transform.localEulerAngles.z;
    }

    speedLerpSound = 5;
    enableEngineSound = false;

    lastRightForwardPositionY = _wheels.rightFrontWheel.wheelMesh.transform.localPosition.y;
    lastLeftForwardPositionY = _wheels.leftFrontWheel.wheelMesh.transform.localPosition.y;
    lastRightRearPositionY = _wheels.rightRearWheel.wheelMesh.transform.localPosition.y;
    lastLeftRearPositionY = _wheels.leftRearWheel.wheelMesh.transform.localPosition.y;

    sensImpactFR = 0.075f * (2.65f * _wheels.rightFrontWheel.wheelCollider.radius);
    sensImpactFL = 0.075f * (2.65f * _wheels.leftFrontWheel.wheelCollider.radius);
    sensImpactRR = 0.075f * (2.65f * _wheels.rightRearWheel.wheelCollider.radius);
    sensImpactRL = 0.075f * (2.65f * _wheels.leftRearWheel.wheelCollider.radius);
  }

  private void SetCameras()
  {
    objStraightStopCameras = new GameObject[_cameras.cameras.Length];
    startRotationCameras = new Quaternion[_cameras.cameras.Length];
    startPositionCameras = new GameObject[_cameras.cameras.Length];
    startETSCamerasPosition = new Vector3[_cameras.cameras.Length];
    xOrbit = new float[_cameras.cameras.Length];
    yOrbit = new float[_cameras.cameras.Length];
    distanceOrbitCamera = new float[_cameras.cameras.Length];
    for (int x = 0; x < _cameras.cameras.Length; x++)
    {
      distanceOrbitCamera[x] = Vector3.Distance(_cameras.cameras[x]._camera.transform.position, transform.position);
      distanceOrbitCamera[x] = Mathf.Clamp(distanceOrbitCamera[x], _cameras.cameraSettings.orbitalCamera.minDistance, _cameras.cameraSettings.orbitalCamera.maxDistance);
      if (!_cameras.cameras[x]._camera)
      {
        Debug.LogError("No camera was associated with variable 'CamerasDoVeiculo.cameras [" + x + "]', therefore an orbital camera will be automatically created in its place.");
        GameObject newCamera = new GameObject("OrbitalCamera" + x) as GameObject;
        newCamera.AddComponent(typeof(Camera));
        newCamera.AddComponent(typeof(FlareLayer));
        //newCamera.AddComponent (typeof(GUILayer));
        newCamera.AddComponent(typeof(AudioListener));
        _cameras.cameras[x]._camera = newCamera.GetComponent<Camera>();
        newCamera.transform.parent = transform;
        newCamera.transform.localPosition = new Vector3(0, 0, 0);
        _cameras.cameras[x].rotationType = CameraTypeClassFree.TipoRotac.Orbital;
      }

      if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.StraightStop)
      {
        objStraightStopCameras[x] = new GameObject("positionCameraStop" + x);
        objStraightStopCameras[x].transform.parent = _cameras.cameras[x]._camera.transform;
        objStraightStopCameras[x].transform.localPosition = new Vector3(0, 0, 1.0f);
        objStraightStopCameras[x].transform.parent = transform;
      }

      if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.FirstPerson)
      {
        startRotationCameras[x] = _cameras.cameras[x]._camera.transform.localRotation;
      }

      if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.FollowPlayer)
      {
        startPositionCameras[x] = new GameObject("positionCameraFollow" + x);
        startPositionCameras[x].transform.parent = transform;
        startPositionCameras[x].transform.position = _cameras.cameras[x]._camera.transform.position;
        if (_cameras.setLayers)
        {
          AjustLayers();
        }
      }

      if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.Orbital)
      {
        xOrbit[x] = _cameras.cameras[x]._camera.transform.eulerAngles.y;
        yOrbit[x] = _cameras.cameras[x]._camera.transform.eulerAngles.x;
        if (_cameras.setLayers)
        {
          AjustLayers();
        }
      }

      if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.OrbitalThatFollows)
      {
        xOrbit[x] = _cameras.cameras[x]._camera.transform.eulerAngles.x;
        yOrbit[x] = _cameras.cameras[x]._camera.transform.eulerAngles.y;
        //
        startPositionCameras[x] = new GameObject("positionCameraFollow" + x);
        startPositionCameras[x].transform.parent = transform;
        startPositionCameras[x].transform.position = _cameras.cameras[x]._camera.transform.position;
        //
        if (_cameras.setLayers)
        {
          AjustLayers();
        }
      }

      if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.ETS_StyleCamera)
      {
        startRotationCameras[x] = _cameras.cameras[x]._camera.transform.localRotation;
        startETSCamerasPosition[x] = _cameras.cameras[x]._camera.transform.localPosition;
      }

      AudioListener _audListner = _cameras.cameras[x]._camera.GetComponent<AudioListener>();
      if (!_audListner)
      {
        _cameras.cameras[x]._camera.transform.gameObject.AddComponent(typeof(AudioListener));
      }

      if (_cameras.cameras[x].volume == 0)
      {
        _cameras.cameras[x].volume = 1;
      }
    }

    if (_cameras.cameras.Length > 0)
    {
      for (int x = 0; x < _cameras.cameras.Length; x++)
      {
        _cameras.cameras[x]._camera.transform.tag = "MainCamera";
        Camera componentCameraX = _cameras.cameras[x]._camera.GetComponent<Camera>();
        if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.LookAtThePlayer)
        {
          componentCameraX.nearClipPlane = 0.5f;
        }

        if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.Orbital)
        {
          componentCameraX.nearClipPlane = 0.5f;
        }

        if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.OrbitalThatFollows)
        {
          componentCameraX.nearClipPlane = 0.5f;
        }

        if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.Stop)
        {
          componentCameraX.nearClipPlane = _cameras.cameraSettings.near;
        }

        if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.StraightStop)
        {
          componentCameraX.nearClipPlane = _cameras.cameraSettings.near;
        }

        if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.FirstPerson)
        {
          componentCameraX.nearClipPlane = _cameras.cameraSettings.near;
        }

        if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.ETS_StyleCamera)
        {
          componentCameraX.nearClipPlane = _cameras.cameraSettings.near;
        }

        if (_cameras.cameras[x].rotationType == CameraTypeClassFree.TipoRotac.FollowPlayer)
        {
          componentCameraX.nearClipPlane = 0.5f;
        }
      }
    }
  }

  private void AjustLayers()
  {
  }

  private void EnableCameras(int nextIndex)
  {
    if (_cameras.cameras.Length > 0)
    {
      if (nextIndex == -1)
      {
        for (int x = 0; x < _cameras.cameras.Length; x++)
        {
          _cameras.cameras[x]._camera.gameObject.SetActive(false);
        }
      }
      else
      {
        for (int x = 0; x < _cameras.cameras.Length; x++)
        {
          if (x == nextIndex)
          {
            _cameras.cameras[x]._camera.gameObject.SetActive(true);
          }
          else
          {
            _cameras.cameras[x]._camera.gameObject.SetActive(false);
          }
        }
      }

      changeTypeCamera = false;
    }
  }

  private void CamerasManager()
  {
/*    timeScaleSpeed = 1.0f / Time.timeScale;

    _cameras.cameras[indexCamera]._camera.transform.position = Vector3.Lerp(_cameras.cameras[indexCamera]._camera.transform.position, startPositionCameras[indexCamera].transform.position, Time.deltaTime * _cameras.cameraSettings.followPlayerCamera.displacementSpeed);

    if (!_cameras.cameraSettings.followPlayerCamera.customLookAt)
    {
      _cameras.cameras[indexCamera]._camera.transform.LookAt(transform);
    }
    else
    {
    newRotationCameras = Quaternion.LookRotation(transform.position - _cameras.cameras[indexCamera]._camera.transform.position, Vector3.up);
    _cameras.cameras[indexCamera]._camera.transform.rotation = Quaternion.Slerp(_cameras.cameras[indexCamera]._camera.transform.rotation, newRotationCameras, Time.deltaTime * _cameras.cameraSettings.followPlayerCamera.spinSpeed);
    }*/
  }


  public static float ClampAngle(float angle, float min, float max)
  {
    if (angle < -360F)
    {
      angle += 360F;
    }

    if (angle > 360F)
    {
      angle -= 360F;
    }

    return Mathf.Clamp(angle, min, max);
  }

  private IEnumerator ShakeCameras(float shakeValue, bool returnStartPosition)
  {
    tempStartPositionShakeCameras = _cameras.cameras[indexCamera]._camera.transform.localPosition;
    _cameras.cameras[indexCamera]._camera.transform.position = new Vector3(_cameras.cameras[indexCamera]._camera.transform.position.x + Random.Range(-shakeValue, shakeValue), _cameras.cameras[indexCamera]._camera.transform.position.y + Random.Range(-shakeValue, shakeValue), _cameras.cameras[indexCamera]._camera.transform.position.z + Random.Range(-shakeValue, shakeValue));
    yield return new WaitForSeconds(0.033f);
    _cameras.cameras[indexCamera]._camera.transform.position = new Vector3(_cameras.cameras[indexCamera]._camera.transform.position.x + Random.Range(-shakeValue, shakeValue), _cameras.cameras[indexCamera]._camera.transform.position.y + Random.Range(-shakeValue, shakeValue), _cameras.cameras[indexCamera]._camera.transform.position.z + Random.Range(-shakeValue, shakeValue));
    yield return new WaitForSeconds(0.033f);
    _cameras.cameras[indexCamera]._camera.transform.position = new Vector3(_cameras.cameras[indexCamera]._camera.transform.position.x + Random.Range(-shakeValue, shakeValue), _cameras.cameras[indexCamera]._camera.transform.position.y + Random.Range(-shakeValue, shakeValue), _cameras.cameras[indexCamera]._camera.transform.position.z + Random.Range(-shakeValue, shakeValue));
    yield return new WaitForSeconds(0.033f);
    if (returnStartPosition)
    {
      _cameras.cameras[indexCamera]._camera.transform.localPosition = tempStartPositionShakeCameras;
    }
  }

  private void Update()
  {
    wheelFDIsGrounded = _wheels.rightFrontWheel.wheelCollider.isGrounded;
    wheelFEIsGrounded = _wheels.leftFrontWheel.wheelCollider.isGrounded;
    wheelTDIsGrounded = _wheels.rightRearWheel.wheelCollider.isGrounded;
    wheelTEIsGrounded = _wheels.leftRearWheel.wheelCollider.isGrounded;

    // INPUT 

    verticalInput = Input.GetAxis("Vertical");
    horizontalInput = Input.GetAxis("Horizontal");

    if (verticalInput > 0.0f || horizontalInput > 0.0f)
    {
      started = true;
    }

    KMh = ms_Rigidbody.velocity.magnitude * 3.6f;
    inclinationFactorForcesDown = Mathf.Clamp(Mathf.Abs(Vector3.Dot(Vector3.up, transform.up)), _vehicleSettings._aerodynamics.downForceAngleFactor, 1.0f);
    if (wheelFDIsGrounded || wheelFEIsGrounded || wheelTDIsGrounded || wheelTEIsGrounded)
    {
      downForceTempLerp = (ms_Rigidbody.mass * _vehicleSettings._aerodynamics.minDownForceValue + (_vehicleSettings._aerodynamics.verticalDownForce * Mathf.Abs(KMh * 3.0f) * (ms_Rigidbody.mass / 125.0f))) * inclinationFactorForcesDown;
      downForceUpdateRef = Mathf.Lerp(downForceUpdateRef, downForceTempLerp, Time.deltaTime * 2.5f);
    }
    else
    {
      downForceTempLerp = ms_Rigidbody.mass * _vehicleSettings._aerodynamics.minDownForceValue * inclinationFactorForcesDown;
      downForceUpdateRef = Mathf.Lerp(downForceUpdateRef, downForceTempLerp, Time.deltaTime * 2.5f);
    }

    ms_Rigidbody.drag = Mathf.Clamp((KMh / _vehicleTorque.maxVelocityKMh) * 0.075f, 0.001f, 0.075f);

    if (!changinGearsAuto)
    {
      engineInput = Mathf.Clamp01(verticalInput);
    }
    else
    {
      engineInput = 0;
    }

    if (isInsideTheCar)
    {
      // INPUT HANDBRAKE
    }

    DiscoverAverageRpm();
    UpdateWheelMeshes();
    if (isInsideTheCar)
    {
      AutomaticGears();
    }
  }

  public void EnterInVehicle()
  {
    isInsideTheCar = true;
    EnableCameras(indexCamera);
  }

  public void ExitTheVehicle()
  {
    isInsideTheCar = false;
    EnableCameras(-1);
    handBrakeTrue = true;
  }

  private void FixedUpdate()
  {
    ApplyTorque();
    Brakes();
    if (isInsideTheCar)
    {
      Volant();
    }

    StabilizeWheelRPM();
    StabilizeVehicleRollForces();
    StabilizeAirRotation();
    StabilizeAngularRotation();
//

//extra gravity
    if (_vehicleSettings._aerodynamics.extraGravity)
    {
      gravityValueFixedUpdate = 0;
      if (wheelFDIsGrounded && wheelFEIsGrounded && wheelTDIsGrounded && wheelTEIsGrounded)
      {
        gravityValueFixedUpdate = 4.0f * ms_Rigidbody.mass * Mathf.Clamp((KMh / _vehicleTorque.maxVelocityKMh), 0.05f, 1.0f);
      }
      else
      {
        gravityValueFixedUpdate = 4.0f * ms_Rigidbody.mass * 3.0f;
      }

      additionalCurrentGravity = Mathf.Lerp(additionalCurrentGravity, gravityValueFixedUpdate, Time.deltaTime);
      ms_Rigidbody.AddForce(Vector3.down * additionalCurrentGravity);
    }

//forcaparaBaixo
    downForceValueFixedUpdate = _vehicleSettings.improveControl.downForce * (((KMh / 10.0f) + 0.3f) / 2.5f);
    currentDownForceVehicle = Mathf.Clamp(Mathf.Lerp(currentDownForceVehicle, downForceValueFixedUpdate, Time.deltaTime * 2.0f), 0.1f, 4.0f);

//forcaparaBaixo2
    ms_Rigidbody.AddForce(-transform.up * downForceUpdateRef);

//tire slips
    SetWheelForces(_wheels.rightFrontWheel.wheelCollider);
    SetWheelForces(_wheels.leftFrontWheel.wheelCollider);
    SetWheelForces(_wheels.rightRearWheel.wheelCollider);
    SetWheelForces(_wheels.leftRearWheel.wheelCollider);

//brakes ABS
    if (wheelFDIsGrounded && wheelFEIsGrounded && wheelTDIsGrounded && wheelTEIsGrounded)
    {
      absSpeedFactor = Mathf.Clamp(KMh, 70, 150);
      if (currentGear > 0 && mediumRPM > 0)
      {
        absBrakeInput = Mathf.Abs(Mathf.Clamp(verticalInput, -1.0f, 0.0f));
      }
      else if (currentGear <= 0 && mediumRPM < 0)
      {
        absBrakeInput = Mathf.Abs(Mathf.Clamp(verticalInput, 0.0f, 1.0f)) * -1;
      }
      else
      {
        absBrakeInput = 0.0f;
      }

      if (isBraking && Mathf.Abs(KMh) > 1.2f)
      {
        ms_Rigidbody.AddForce(-transform.forward * absSpeedFactor * ms_Rigidbody.mass * 0.125f * absBrakeInput);
      }
    }
  }

  #region UpdateWheelMesh

  private void UpdateWheelMeshes()
  {
    _wheels.rightFrontWheel.wheelCollider.GetWorldPose(out vectorMeshPos1, out quatMesh1);
    _wheels.rightFrontWheel.wheelWorldPosition = _wheels.rightFrontWheel.wheelMesh.position = vectorMeshPos1;
    _wheels.rightFrontWheel.wheelMesh.rotation = quatMesh1;
//
    _wheels.leftFrontWheel.wheelCollider.GetWorldPose(out vectorMeshPos2, out quatMesh2);
    _wheels.leftFrontWheel.wheelWorldPosition = _wheels.leftFrontWheel.wheelMesh.position = vectorMeshPos2;
    _wheels.leftFrontWheel.wheelMesh.rotation = quatMesh2;
//
    _wheels.rightRearWheel.wheelCollider.GetWorldPose(out vectorMeshPos3, out quatMesh3);
    _wheels.rightRearWheel.wheelWorldPosition = _wheels.rightRearWheel.wheelMesh.position = vectorMeshPos3;
    _wheels.rightRearWheel.wheelMesh.rotation = quatMesh3;
//
    _wheels.leftRearWheel.wheelCollider.GetWorldPose(out vectorMeshPos4, out quatMesh4);
    _wheels.leftRearWheel.wheelWorldPosition = _wheels.leftRearWheel.wheelMesh.position = vectorMeshPos4;
    _wheels.leftRearWheel.wheelMesh.rotation = quatMesh4;
  }

  #endregion

  private void LateUpdate()
  {
    if (enableSkidMarksOnStart)
    {
      CheckGroundForSKidMarks();
    }

    if (_cameras.cameras.Length > 0 && Time.timeScale > 0.2f)
    {
      CamerasManager();
    }
  }

  private void DiscoverAverageRpm()
  {
    groundedWheels = 0;
    sumRPM = 0;
    _wheels.rightFrontWheel.wheelColliderRPM = _wheels.rightFrontWheel.wheelCollider.rpm;
    if (wheelFDIsGrounded)
    {
      groundedWheels++;
      sumRPM += _wheels.rightFrontWheel.wheelColliderRPM;
    }

//
    _wheels.leftFrontWheel.wheelColliderRPM = _wheels.leftFrontWheel.wheelCollider.rpm;
    if (wheelFEIsGrounded)
    {
      groundedWheels++;
      sumRPM += _wheels.leftFrontWheel.wheelColliderRPM;
    }

//
    _wheels.rightRearWheel.wheelColliderRPM = _wheels.rightRearWheel.wheelCollider.rpm;
    if (wheelTDIsGrounded)
    {
      groundedWheels++;
      sumRPM += _wheels.rightRearWheel.wheelColliderRPM;
    }

//
    _wheels.leftRearWheel.wheelColliderRPM = _wheels.leftRearWheel.wheelCollider.rpm;
    if (wheelTEIsGrounded)
    {
      groundedWheels++;
      sumRPM += _wheels.leftRearWheel.wheelColliderRPM;
    }

    mediumRPM = sumRPM / groundedWheels;
    if (Mathf.Abs(mediumRPM) < 0.01f)
    {
      mediumRPM = 0.0f;
    }
  }

  #region tireForces

  private void SetWheelForces(WheelCollider wheelCollider)
  {
    wheelCollider.GetGroundHit(out tempWheelHit);
    if (wheelCollider.isGrounded)
    {
      TireSlips(wheelCollider, tempWheelHit);
      distanceXForceTemp = ms_Rigidbody.centerOfMass.y - transform.InverseTransformPoint(wheelCollider.transform.position).y + wheelCollider.radius + (1.0f - wheelCollider.suspensionSpring.targetPosition) * wheelCollider.suspensionDistance;
      lateralForcePointTemp = tempWheelHit.point + wheelCollider.transform.up * _vehicleSettings.improveControl.helpToStraightenOut * distanceXForceTemp;
      forwardForceTemp = tempWheelHit.forwardDir * (tireFO.y) * 3.0f;
      lateralForceTemp = tempWheelHit.sidewaysDir * (tireFO.x);
      if (Mathf.Abs(horizontalInput) > 0.1f && wheelCollider.steerAngle != 0.0f && Mathf.Sign(wheelCollider.steerAngle) != Mathf.Sign(tireSL.x))
      {
        lateralForcePointTemp += tempWheelHit.forwardDir * _vehicleSettings.improveControl.helpToTurn;
      }

      ms_Rigidbody.AddForceAtPosition(forwardForceTemp, tempWheelHit.point);
      ms_Rigidbody.AddForceAtPosition(lateralForceTemp, lateralForcePointTemp);
    }
  }

  public Vector2 WheelLocalVelocity(WheelHit wheelHit)
  {
    tempLocalVelocityVector2 = new Vector2(0, 0);
    tempWheelVelocityVector3 = ms_Rigidbody.GetPointVelocity(wheelHit.point);
    velocityLocalWheelTemp = tempWheelVelocityVector3 - Vector3.Project(tempWheelVelocityVector3, wheelHit.normal);
    tempLocalVelocityVector2.y = Vector3.Dot(wheelHit.forwardDir, velocityLocalWheelTemp);
    tempLocalVelocityVector2.x = Vector3.Dot(wheelHit.sidewaysDir, velocityLocalWheelTemp);
    return tempLocalVelocityVector2;
  }

  public float AngularVelocity(Vector2 localVelocityVector, WheelCollider wheelCollider)
  {
    wheelCollider.GetGroundHit(out tempWheelHit);
    return (localVelocityVector.y + (tempWheelHit.sidewaysSlip * ((Mathf.Abs(verticalInput) + Mathf.Abs(horizontalInput)) / 2.0f) * (-2.0f))) / wheelCollider.radius;
  }

  public Vector2 LocalSurfaceForce(WheelHit wheelHit)
  {
    wheelSpeedLocalSurface = ms_Rigidbody.GetPointVelocity(wheelHit.point);
    forceFactorTempLocalSurface = Mathf.InverseLerp(1.0f, 0.25f, (wheelSpeedLocalSurface - Vector3.Project(wheelSpeedLocalSurface, wheelHit.normal)).sqrMagnitude);
    if (forceFactorTempLocalSurface > 0.0f)
    {
      normalTemp = Vector3.Dot(Vector3.up, wheelHit.normal);
      if (normalTemp > 0.000001f)
      {
        downForceUPTemp = Vector3.up * wheelHit.force / normalTemp;
        surfaceLocalForceTemp = downForceUPTemp - Vector3.Project(downForceUPTemp, wheelHit.normal);
      }
      else
      {
        surfaceLocalForceTemp = Vector3.up * 1000000.0f;
      }

      surfaceLocalForce.y = Vector3.Dot(wheelHit.forwardDir, surfaceLocalForceTemp);
      surfaceLocalForce.x = Vector3.Dot(wheelHit.sidewaysDir, surfaceLocalForceTemp);
      surfaceLocalForce *= forceFactorTempLocalSurface;
    }
    else
    {
      surfaceLocalForce = Vector2.zero;
    }

    return surfaceLocalForce;
  }

  public void TireSlips(WheelCollider wheelCollider, WheelHit wheelHit)
  {
    localVelocityWheelTireSlips = WheelLocalVelocity(wheelHit);
    localSurfaceForceDTireSlips = LocalSurfaceForce(wheelHit);
    if (KMh > _vehicleTorque.maxVelocityKMh)
    {
      reverseForce = -5 * ms_Rigidbody.velocity.magnitude;
    }
    else
    {
      reverseForce = 0;
    }

    angularWheelVelocityTireSlips = AngularVelocity(localVelocityWheelTireSlips, wheelCollider);
    if (wheelCollider.isGrounded)
    {
      estimatedSprungMass = Mathf.Clamp(wheelHit.force / -Physics.gravity.y, 0.0f, wheelCollider.sprungMass) * 0.5f;
      localRigForceTireSlips = (-estimatedSprungMass * localVelocityWheelTireSlips / Time.deltaTime) + localSurfaceForceDTireSlips;
      tireSlipTireSlips.x = localVelocityWheelTireSlips.x;
      tireSlipTireSlips.y = localVelocityWheelTireSlips.y - angularWheelVelocityTireSlips * wheelCollider.radius;
      downForceTireSlips = (currentDownForceVehicle * _vehicleSettings.vehicleMass);
      if (wheelCollider.brakeTorque > 10)
      {
        wheelMaxBrakeSlip = Mathf.Max(Mathf.Abs(localVelocityWheelTireSlips.y * 0.2f), 0.3f);
        minSlipYTireSlips = Mathf.Clamp(Mathf.Abs(reverseForce * tireSlipTireSlips.x) / downForceTireSlips, 0.0f, wheelMaxBrakeSlip);
      }
      else
      {
        minSlipYTireSlips = Mathf.Min(Mathf.Abs(reverseForce * tireSlipTireSlips.x) / downForceTireSlips, Mathf.Clamp((verticalInput * 2.5f), -2.5f, 1.0f));
        if (reverseForce != 0.0f && minSlipYTireSlips < 0.1f) minSlipYTireSlips = 0.1f;
      }

      if (Mathf.Abs(tireSlipTireSlips.y) < minSlipYTireSlips) tireSlipTireSlips.y = minSlipYTireSlips * Mathf.Sign(tireSlipTireSlips.y);
      rawTireForceTireSlips = -downForceTireSlips * tireSlipTireSlips.normalized;
      rawTireForceTireSlips.x = Mathf.Abs(rawTireForceTireSlips.x);
      rawTireForceTireSlips.y = Mathf.Abs(rawTireForceTireSlips.y);
      tireForceTireSlips.x = Mathf.Clamp(localRigForceTireSlips.x, -rawTireForceTireSlips.x, +rawTireForceTireSlips.x);
      if (wheelCollider.brakeTorque > 10)
      {
        maxFyTireSlips = Mathf.Min(rawTireForceTireSlips.y, reverseForce);
        tireForceTireSlips.y = Mathf.Clamp(localRigForceTireSlips.y, -maxFyTireSlips, +maxFyTireSlips);
      }
      else
      {
        tireForceTireSlips.y = Mathf.Clamp(reverseForce, -rawTireForceTireSlips.y, +rawTireForceTireSlips.y);
      }
    }
    else
    {
      tireSlipTireSlips = Vector2.zero;
      tireForceTireSlips = Vector2.zero;
    }

    tireSL = tireSlipTireSlips * _vehicleSettings.improveControl.tireSlipsFactor;
    tireFO = tireForceTireSlips * _vehicleSettings.improveControl.tireSlipsFactor;
  }

  #endregion

  private void OnCollisionStay()
  {
    colliding = true;
  }

  private void OnCollisionExit()
  {
    colliding = false;
  }

  #region Stabilizers

  private void StabilizeAngularRotation()
  {
    if (Mathf.Abs(horizontalInput) < 0.9f)
    {
      ms_Rigidbody.angularVelocity = Vector3.Lerp(ms_Rigidbody.angularVelocity, new Vector3(ms_Rigidbody.angularVelocity.x, 0, ms_Rigidbody.angularVelocity.z), Time.deltaTime * 2);
    }
  }

  private void StabilizeAirRotation()
  {
    if (!colliding)
    {
      isGroundedExtraW = false;
      if (!wheelFDIsGrounded && !wheelFEIsGrounded && !wheelTDIsGrounded && !wheelTEIsGrounded && !isGroundedExtraW)
      {
        axisFromRotate = Vector3.Cross(transform.up, Vector3.up);
        torqueForceAirRotation = axisFromRotate.normalized * axisFromRotate.magnitude * 2.0f;
        torqueForceAirRotation -= ms_Rigidbody.angularVelocity;
        ms_Rigidbody.AddTorque(torqueForceAirRotation * ms_Rigidbody.mass * 0.02f, ForceMode.Impulse);
        if (Mathf.Abs(horizontalInput) > 0.1f)
        {
          ms_Rigidbody.AddTorque(transform.forward * -horizontalInput * _vehicleSettings.vehicleMass * 0.6f);
        }

        if (Mathf.Abs(verticalInput) > 0.1f)
        {
          ms_Rigidbody.AddTorque(transform.right * verticalInput * _vehicleSettings.vehicleMass * 0.44f);
        }
      }
    }
  }

  private void StabilizeWheelRPM()
  {
    if (currentGear > 0)
    {
      if (KMh > (_vehicleTorque.maxVelocityGears[currentGear - 1] * _vehicleTorque.speedOfGear) && Mathf.Abs(verticalInput) < 0.5f)
      {
        if (_wheels.rightFrontWheel.wheelDrive)
        {
          _wheels.rightFrontWheel.wheelCollider.brakeTorque = forceEngineBrake;
        }

        if (_wheels.leftFrontWheel.wheelDrive)
        {
          _wheels.leftFrontWheel.wheelCollider.brakeTorque = forceEngineBrake;
        }

        if (_wheels.rightRearWheel.wheelDrive)
        {
          _wheels.rightRearWheel.wheelCollider.brakeTorque = forceEngineBrake;
        }

        if (_wheels.leftRearWheel.wheelDrive)
        {
          _wheels.leftRearWheel.wheelCollider.brakeTorque = forceEngineBrake;
        }
      }
    }
    else if (currentGear == -1)
    {
      if (KMh > (_vehicleTorque.maxVelocityGears[0] * _vehicleTorque.speedOfGear) && Mathf.Abs(verticalInput) < 0.5f)
      {
        if (_wheels.rightFrontWheel.wheelDrive)
        {
          _wheels.rightFrontWheel.wheelCollider.brakeTorque = forceEngineBrake / 5.0f;
        }

        if (_wheels.leftFrontWheel.wheelDrive)
        {
          _wheels.leftFrontWheel.wheelCollider.brakeTorque = forceEngineBrake / 5.0f;
        }

        if (_wheels.rightRearWheel.wheelDrive)
        {
          _wheels.rightRearWheel.wheelCollider.brakeTorque = forceEngineBrake / 5.0f;
        }

        if (_wheels.leftRearWheel.wheelDrive)
        {
          _wheels.leftRearWheel.wheelCollider.brakeTorque = forceEngineBrake / 5.0f;
        }
      }
    }
  }

  private void StabilizeVehicleRollForces()
  {
    leftFrontForce = 1.0f;
    rightFrontForce = 1.0f;
    leftRearForce = 1.0f;
    rightRearForce = 1.0f;
//CHECAR COLISOES
//rodasTraz
    bool isGround1 = _wheels.leftRearWheel.wheelCollider.GetGroundHit(out tempWheelHit);
    if (isGround1)
    {
      leftRearForce = (-_wheels.leftRearWheel.wheelCollider.transform.InverseTransformPoint(tempWheelHit.point).y - _wheels.leftRearWheel.wheelCollider.radius) / _wheels.leftRearWheel.wheelCollider.suspensionDistance;
    }

    bool isGround2 = _wheels.rightRearWheel.wheelCollider.GetGroundHit(out tempWheelHit);
    if (isGround2)
    {
      rightRearForce = (-_wheels.rightRearWheel.wheelCollider.transform.InverseTransformPoint(tempWheelHit.point).y - _wheels.rightRearWheel.wheelCollider.radius) / _wheels.rightRearWheel.wheelCollider.suspensionDistance;
    }

//rodasFrente
    bool isGround3 = _wheels.leftFrontWheel.wheelCollider.GetGroundHit(out tempWheelHit);
    if (isGround3)
    {
      leftFrontForce = (-_wheels.leftFrontWheel.wheelCollider.transform.InverseTransformPoint(tempWheelHit.point).y - _wheels.leftFrontWheel.wheelCollider.radius) / _wheels.leftFrontWheel.wheelCollider.suspensionDistance;
    }

    bool isGround4 = _wheels.rightFrontWheel.wheelCollider.GetGroundHit(out tempWheelHit);
    if (isGround4)
    {
      rightFrontForce = (-_wheels.rightFrontWheel.wheelCollider.transform.InverseTransformPoint(tempWheelHit.point).y - _wheels.rightFrontWheel.wheelCollider.radius) / _wheels.rightFrontWheel.wheelCollider.suspensionDistance;
    }

//APLICAR FORCAS DESCOBERTAS
    roolForce1 = (leftRearForce - rightRearForce) * _vehicleSettings._aerodynamics.feelingHeavy * _vehicleSettings.vehicleMass * inclinationFactorForcesDown;
    roolForce2 = (leftFrontForce - rightFrontForce) * _vehicleSettings._aerodynamics.feelingHeavy * _vehicleSettings.vehicleMass * inclinationFactorForcesDown;
//rodasTraz
    if (isGround1)
    {
      ms_Rigidbody.AddForceAtPosition(_wheels.leftRearWheel.wheelCollider.transform.up * -roolForce1, _wheels.leftRearWheel.wheelCollider.transform.position);
    }

    if (isGround2)
    {
      ms_Rigidbody.AddForceAtPosition(_wheels.rightRearWheel.wheelCollider.transform.up * roolForce1, _wheels.rightRearWheel.wheelCollider.transform.position);
    }

//rodasFrente
    if (isGround3)
    {
      ms_Rigidbody.AddForceAtPosition(_wheels.leftFrontWheel.wheelCollider.transform.up * -roolForce2, _wheels.leftFrontWheel.wheelCollider.transform.position);
    }

    if (isGround4)
    {
      ms_Rigidbody.AddForceAtPosition(_wheels.rightFrontWheel.wheelCollider.transform.up * roolForce2, _wheels.rightFrontWheel.wheelCollider.transform.position);
    }
  }

  #endregion

  #region GearsManager

  private void AutomaticGears()
  {
//aqui
    if (currentGear == 0)
    {
//entre -5 e 5 RPM, se a marcha estver em 0
      if (mediumRPM < 5 && mediumRPM >= 0)
      {
        currentGear = 1;
      }

      if (mediumRPM > -5 && mediumRPM < 0)
      {
        currentGear = -1;
      }
    }

    if (mediumRPM < -0.1f && Mathf.Abs(verticalInput) < 0.1f)
    {
      currentGear = -1;
    }

    if (Mathf.Abs(verticalInput) < 0.1f && mediumRPM >= 0 && currentGear < 2)
    {
      currentGear = 1;
    }

    if ((Mathf.Abs(Mathf.Clamp(verticalInput, -1f, 0f))) > 0.8f)
    {
      if ((KMh < 5 && mediumRPM < 1) || mediumRPM < -2)
      {
        currentGear = -1;
      }
    }

    if ((Mathf.Abs(Mathf.Clamp(verticalInput, 0f, 1f))) > 0.8f)
    {
      if ((KMh < 5) || (mediumRPM > 2 && currentGear < 2))
      {
        currentGear = 1;
      }
    }

    if (currentGear > 0)
    {
      if (KMh > (_vehicleTorque.idealVelocityGears[currentGear - 1] * _vehicleTorque.speedOfGear + 7 * _vehicleTorque.speedOfGear))
      {
        if (currentGear < _vehicleTorque.numberOfGears && !changinGearsAuto && currentGear != -1)
        {
          timeAutoGear = 1.5f;
          StartCoroutine(nameof(TimeAutoGears), currentGear + 1);
        }
      }
      else if (KMh < (_vehicleTorque.idealVelocityGears[currentGear - 1] * _vehicleTorque.speedOfGear - 15 * _vehicleTorque.speedOfGear))
      {
        if (currentGear > 1 && !changinGearsAuto)
        {
          timeAutoGear = 0;
          StartCoroutine(nameof(TimeAutoGears), currentGear - 1);
        }
      }

      if (verticalInput > 0.1f && KMh > (_vehicleTorque.idealVelocityGears[currentGear - 1] * _vehicleTorque.speedOfGear + 7 * _vehicleTorque.speedOfGear))
      {
        if (currentGear < _vehicleTorque.numberOfGears && currentGear != -1)
        {
          timeAutoGear = 0.0f;
          StartCoroutine(nameof(TimeAutoGears), currentGear + 1);
        }
      }
    }
  }

  private IEnumerator TimeAutoGears(int gear)
  {
    changinGearsAuto = true;
    yield return new WaitForSeconds(0.4f);
    currentGear = gear;
    yield return new WaitForSeconds(timeAutoGear);
    changinGearsAuto = false;
  }

  #endregion

  #region VolantManager

  private void Volant()
  {
    angle1Ref = Mathf.MoveTowards(angle1Ref, horizontalInput, 2 * Time.deltaTime);
    angle2Volant = Mathf.MoveTowards(angle2Volant, horizontalInput, 2 * Time.deltaTime);
    maxAngleVolant = 27.0f * _vehicleSettings.angle_x_Velocity.Evaluate(KMh);
    angleRefVolant = Mathf.Clamp(angle1Ref * maxAngleVolant, -maxAngleVolant, maxAngleVolant);

//APLICAR ANGULO NAS RODAS--------------------------------------------------------------------------------------------------------------
    if (angle1Ref > 0.2f)
    {
      if (_wheels.rightFrontWheel.wheelTurn)
      {
        _wheels.rightFrontWheel.wheelCollider.steerAngle = angleRefVolant * 1.2f;
      }

      if (_wheels.leftFrontWheel.wheelTurn)
      {
        _wheels.leftFrontWheel.wheelCollider.steerAngle = angleRefVolant;
      }

      if (_wheels.rightRearWheel.wheelTurn)
      {
        _wheels.rightRearWheel.wheelCollider.steerAngle = angleRefVolant * 1.2f;
      }

      if (_wheels.leftRearWheel.wheelTurn)
      {
        _wheels.leftRearWheel.wheelCollider.steerAngle = angleRefVolant;
      }
    }
    else if (angle1Ref < -0.2f)
    {
      if (_wheels.rightFrontWheel.wheelTurn)
      {
        _wheels.rightFrontWheel.wheelCollider.steerAngle = angleRefVolant;
      }

      if (_wheels.leftFrontWheel.wheelTurn)
      {
        _wheels.leftFrontWheel.wheelCollider.steerAngle = angleRefVolant * 1.2f;
      }

      if (_wheels.rightRearWheel.wheelTurn)
      {
        _wheels.rightRearWheel.wheelCollider.steerAngle = angleRefVolant;
      }

      if (_wheels.leftRearWheel.wheelTurn)
      {
        _wheels.leftRearWheel.wheelCollider.steerAngle = angleRefVolant * 1.2f;
      }
    }
    else
    {
      if (_wheels.rightFrontWheel.wheelTurn)
      {
        _wheels.rightFrontWheel.wheelCollider.steerAngle = angleRefVolant;
      }

      if (_wheels.leftFrontWheel.wheelTurn)
      {
        _wheels.leftFrontWheel.wheelCollider.steerAngle = angleRefVolant;
      }

      if (_wheels.rightRearWheel.wheelTurn)
      {
        _wheels.rightRearWheel.wheelCollider.steerAngle = angleRefVolant;
      }

      if (_wheels.leftRearWheel.wheelTurn)
      {
        _wheels.leftRearWheel.wheelCollider.steerAngle = angleRefVolant;
      }
    }

    if (_vehicleSettings.volant)
    {
      if (_vehicleSettings.volant)
      {
        _vehicleSettings.volant.transform.localEulerAngles = new Vector3(_vehicleSettings.volant.transform.localEulerAngles.x, _vehicleSettings.volant.transform.localEulerAngles.y, volantStartRotation + (angle2Volant * 320.0f));
      }
    }
  }

  #endregion

  #region UpdateTorque

  public float VehicleTorque(WheelCollider wheelCollider)
  {
    torqueM = 0;
    rpmTempTorque = Mathf.Abs(wheelCollider.rpm);
    if (!isInsideTheCar)
    {
      return 0;
    }

    if ((Mathf.Abs(verticalInput) < 0.5f) || KMh > _vehicleTorque.maxVelocityKMh)
    {
      return 0;
    }

    if ((rpmTempTorque * wheelCollider.radius) > (50.0f * _vehicleTorque.numberOfGears * _vehicleTorque.speedOfGear))
    {
      return 0;
    }

    if (KMh < 0.5f)
    {
      if (rpmTempTorque > (25.0f / wheelCollider.radius))
      {
        return 0;
      }
    }

    if (!theEngineIsRunning)
    {
      return 0;
    }

    if (handBrakeTrue)
    {
      return 0;
    }

    if (isBraking)
    {
      return 0;
    }

    if (currentBrakeValue > 0.1f)
    {
      return 0;
    }

    if (currentGear < 0)
    {
      clampInputTorque = Mathf.Abs(Mathf.Clamp(verticalInput, -1f, 0f));
      torqueM = (500.0f * _vehicleTorque.engineTorque) * clampInputTorque * (_vehicleTorque.gears[0].Evaluate((KMh / _vehicleTorque.speedOfGear))) * -0.8f;
    }
    else if (currentGear == 0)
    {
      return 0;
    }
    else
    {
      torqueM = (500.0f * _vehicleTorque.engineTorque) * (Mathf.Clamp(engineInput, 0f, 1f)) * _vehicleTorque.gears[currentGear - 1].Evaluate((KMh / _vehicleTorque.speedOfGear));
    }

//AJUSTE MANUAL DAS MARCHAS
    adjustTorque = 1;
    if (currentGear < _vehicleTorque.manualAdjustmentOfTorques.Length && currentGear > 0)
    {
      if (currentGear == -1)
      {
        adjustTorque = _vehicleTorque.manualAdjustmentOfTorques[0];
      }
      else if (currentGear == 0)
      {
        adjustTorque = 0;
      }
      else if (currentGear > 0)
      {
        adjustTorque = _vehicleTorque.manualAdjustmentOfTorques[currentGear - 1];
      }
    }
    else
    {
      adjustTorque = 1;
    }

    print(torqueM * adjustTorque * vehicleScale);

    return torqueM * adjustTorque * vehicleScale;
  }

  private void ApplyTorque()
  {
    leftDifferential = 1 + Mathf.Abs((0.2f * Mathf.Abs(Mathf.Clamp(horizontalInput, 0, 1))) * (angleRefVolant / 60));
    rightDifferential = 1 + Mathf.Abs((0.2f * Mathf.Abs(Mathf.Clamp(horizontalInput, -1, 0))) * (angleRefVolant / 60));
//torque do motor
    if (theEngineIsRunning && isInsideTheCar)
    {
      if (_wheels.rightFrontWheel.wheelDrive)
      {
        _wheels.rightFrontWheel.wheelCollider.motorTorque = VehicleTorque(_wheels.rightFrontWheel.wheelCollider) * rightDifferential;
      }

      if (_wheels.leftFrontWheel.wheelDrive)
      {
        _wheels.leftFrontWheel.wheelCollider.motorTorque = VehicleTorque(_wheels.leftFrontWheel.wheelCollider) * leftDifferential;
      }

      if (_wheels.rightRearWheel.wheelDrive)
      {
        _wheels.rightRearWheel.wheelCollider.motorTorque = VehicleTorque(_wheels.rightRearWheel.wheelCollider) * rightDifferential;
      }

      if (_wheels.leftRearWheel.wheelDrive)
      {
        _wheels.leftRearWheel.wheelCollider.motorTorque = VehicleTorque(_wheels.leftRearWheel.wheelCollider) * leftDifferential;
      }
    }
    else
    {
      if (_wheels.rightFrontWheel.wheelDrive)
      {
        _wheels.rightFrontWheel.wheelCollider.motorTorque = 0;
      }

      if (_wheels.leftFrontWheel.wheelDrive)
      {
        _wheels.leftFrontWheel.wheelCollider.motorTorque = 0;
      }

      if (_wheels.rightRearWheel.wheelDrive)
      {
        _wheels.rightRearWheel.wheelCollider.motorTorque = 0;
      }

      if (_wheels.leftRearWheel.wheelDrive)
      {
        _wheels.leftRearWheel.wheelCollider.motorTorque = 0;
      }
    }
  }

  #endregion

  #region BrakesUpdate

  private void Brakes()
  {
    brakeVerticalInput = 0.0f;
    if (isInsideTheCar)
    {
      brakeVerticalInput = verticalInput;
    }

//Freio de pé
    if (currentGear > 0)
    {
      currentBrakeValue = Mathf.Abs(Mathf.Clamp(brakeVerticalInput, -1.0f, 0.0f)) * 1.5f;
    }
    else if (currentGear < 0)
    {
      currentBrakeValue = Mathf.Abs(Mathf.Clamp(brakeVerticalInput, 0.0f, 1.0f)) * 1.5f;
    }
    else if (currentGear == 0)
    {
      if (mediumRPM > 0)
      {
        currentBrakeValue = Mathf.Abs(Mathf.Clamp(brakeVerticalInput, -1.0f, 0.0f)) * 1.5f;
      }
      else
      {
        currentBrakeValue = Mathf.Abs(Mathf.Clamp(brakeVerticalInput, 0.0f, 1.0f)) * 1.5f;
      }
    }

// FREIO DE MÃO
    handBrake_Input = 0.0f;
    if (handBrakeTrue)
    {
      if (Mathf.Abs(brakeVerticalInput) < 0.9f)
      {
        handBrake_Input = 2;
      }
      else
      {
        handBrake_Input = 0;
        handBrakeTrue = false;
      }
    }
    else
    {
      handBrake_Input = 0;
    }

    handBrake_Input = handBrake_Input * 1000;
//FREIO TOTAL
    totalFootBrake = currentBrakeValue * 0.5f * _vehicleSettings.vehicleMass;
    totalHandBrake = handBrake_Input * 0.5f * _vehicleSettings.vehicleMass;
    if (isInsideTheCar)
    {
      if (Mathf.Abs(mediumRPM) < 15 && Mathf.Abs(brakeVerticalInput) < 0.05f && !handBrakeTrue && (totalFootBrake + totalHandBrake) < 100)
      {
        brakingAuto = true;
        totalFootBrake = 1.5f * _vehicleSettings.vehicleMass;
      }
      else
      {
        brakingAuto = false;
      }
    }
    else
    {
      brakingAuto = false;
    }

//freiar\/
    if (totalFootBrake > 10)
    {
      isBraking = true;
    }
    else
    {
      isBraking = false;
    }

    if (!brakingAuto)
    {
      if (isBraking && Mathf.Abs(KMh) > 1.2f)
      {
        totalFootBrake = 0;
      }
    }

    ApplyBrakeInWheels(_wheels.rightFrontWheel.wheelCollider, _wheels.rightFrontWheel.wheelHandBrake);
    ApplyBrakeInWheels(_wheels.leftFrontWheel.wheelCollider, _wheels.leftFrontWheel.wheelHandBrake);
    ApplyBrakeInWheels(_wheels.rightRearWheel.wheelCollider, _wheels.rightRearWheel.wheelHandBrake);
    ApplyBrakeInWheels(_wheels.leftRearWheel.wheelCollider, _wheels.leftRearWheel.wheelHandBrake);
  }

  private void ApplyBrakeInWheels(WheelCollider wheelCollider, bool handBrake)
  {
    if (handBrake)
    {
      wheelCollider.brakeTorque = totalFootBrake + totalHandBrake;
    }
    else
    {
      wheelCollider.brakeTorque = totalFootBrake;
    }

//evitar RPM, freio ou torques invalidos, EvitarRotacaoSemTorque
    if (!wheelCollider.isGrounded && Mathf.Abs(wheelCollider.rpm) > 0.5f && Mathf.Abs(verticalInput) < 0.05f && wheelCollider.motorTorque < 5.0f)
    {
      wheelCollider.brakeTorque += _vehicleSettings.vehicleMass * Time.deltaTime * 50;
    }

    if (KMh < 0.5f && Mathf.Abs(verticalInput) < 0.05f)
    {
      if (wheelCollider.rpm > (25 / wheelCollider.radius))
      {
        wheelCollider.brakeTorque += 0.5f * _vehicleSettings.vehicleMass * Mathf.Abs(wheelCollider.rpm) * Time.deltaTime;
      }
    }
  }

  #endregion

  #region skidMarksGeneration

  private void CheckGroundForSKidMarks()
  {
    if (_wheels.rightFrontWheel.wheelCollider)
    {
      if (wheelFDIsGrounded)
      {
        _wheels.rightFrontWheel.generateSkidBool = GenerateSkidMarks(_wheels.rightFrontWheel.wheelCollider, _wheels.rightFrontWheel.wheelWorldPosition,
          _wheels.rightFrontWheel.rendSKDmarks, _wheels.rightFrontWheel.generateSkidBool, _wheels.rightFrontWheel.skidMarkShift, 0);
      }
      else
      {
        _wheels.rightFrontWheel.generateSkidBool = false;
      }
    }

//
    if (_wheels.leftFrontWheel.wheelCollider)
    {
      if (wheelFEIsGrounded)
      {
        _wheels.leftFrontWheel.generateSkidBool = GenerateSkidMarks(_wheels.leftFrontWheel.wheelCollider, _wheels.leftFrontWheel.wheelWorldPosition,
          _wheels.leftFrontWheel.rendSKDmarks, _wheels.leftFrontWheel.generateSkidBool, _wheels.leftFrontWheel.skidMarkShift, 1);
      }
      else
      {
        _wheels.leftFrontWheel.generateSkidBool = false;
      }
    }

//
    if (_wheels.rightRearWheel.wheelCollider)
    {
      if (wheelTDIsGrounded)
      {
        _wheels.rightRearWheel.generateSkidBool = GenerateSkidMarks(_wheels.rightRearWheel.wheelCollider, _wheels.rightRearWheel.wheelWorldPosition,
          _wheels.rightRearWheel.rendSKDmarks, _wheels.rightRearWheel.generateSkidBool, _wheels.rightRearWheel.skidMarkShift, 2);
      }
      else
      {
        _wheels.rightRearWheel.generateSkidBool = false;
      }
    }

//
    if (_wheels.leftRearWheel.wheelCollider)
    {
      if (wheelTEIsGrounded)
      {
        _wheels.leftRearWheel.generateSkidBool = GenerateSkidMarks(_wheels.leftRearWheel.wheelCollider, _wheels.leftRearWheel.wheelWorldPosition,
          _wheels.leftRearWheel.rendSKDmarks, _wheels.leftRearWheel.generateSkidBool, _wheels.leftRearWheel.skidMarkShift, 3);
      }
      else
      {
        _wheels.leftRearWheel.generateSkidBool = false;
      }
    }
  }

  private int GetCurrentVerticeIndexForMesh(Mesh mesh)
  {
    int result;
    currentIndexes.TryGetValue(mesh, out result);
    result += 2;
    result %= mesh.vertexCount;
    result += mesh.vertexCount;
    currentIndexes[mesh] = result;
    return result;
  }

  private static T GetRepeatedArrayValue<T>(List<T> array, int index)
  {
    return array[GetRepeatedArrayIndex(array, index)];
  }

  private static int GetRepeatedArrayIndex<T>(List<T> array, int index)
  {
    return (index + array.Count) % array.Count;
  }

  private bool GenerateSkidMarks(WheelCollider wheelCollider, Vector3 wheelPos, Mesh wheelMesh, bool generateBool, float lateralDisplacement, int indexLastMark)
  {
    if (!wheelCollider.GetGroundHit(out tempWHeelHit) || !wheelCollider.isGrounded)
    {
      return false;
    }

    tempWHeelHit.point = wheelPos - wheelCollider.transform.up * wheelCollider.radius * vehicleScale;
    tempAlphaSkidMarks = Mathf.Abs(tempWHeelHit.sidewaysSlip);
    if (Mathf.Abs(tempWHeelHit.forwardSlip) > tempAlphaSkidMarks)
    {
      tempAlphaSkidMarks = Mathf.Abs(tempWHeelHit.forwardSlip);
    }

//
    skidTemp = tempWHeelHit.sidewaysDir * (_skidMarks.standardBrandWidth * vehicleScale) / 2f * Vector3.Dot(wheelCollider.attachedRigidbody.velocity.normalized, tempWHeelHit.forwardDir);
    skidTemp -= tempWHeelHit.forwardDir * (_skidMarks.standardBrandWidth * vehicleScale) * 0.1f * Vector3.Dot(wheelCollider.attachedRigidbody.velocity.normalized, tempWHeelHit.sidewaysDir);
    if (KMh > (75.0f / _skidMarks.sensibility) && Mathf.Abs(wheelCollider.rpm) < (3.0f / _skidMarks.sensibility))
    {
      if (wheelCollider.isGrounded)
      {
        tempAlphaSkidMarks = 10;
      }
    }

    if (KMh < 20.0f * (Mathf.Clamp(_skidMarks.sensibility, 1, 3)))
    {
      if (Mathf.Abs(tempWHeelHit.forwardSlip) > (1.2f / _skidMarks.sensibility))
      {
        if (wheelCollider.isGrounded)
        {
          tempAlphaSkidMarks = 10;
        }
      }
    }

    if (Mathf.Abs(wheelCollider.rpm) < 5 && KMh > 5)
    {
      if (wheelCollider.isGrounded)
      {
        tempAlphaSkidMarks = 10;
      }
    }

    if (tempAlphaSkidMarks < (1 / _skidMarks.sensibility))
    {
      return false;
    }

    float distance = (lastPoint[indexLastMark] - tempWHeelHit.point - skidTemp).sqrMagnitude;
    float alphaAplic = Mathf.Clamp(tempAlphaSkidMarks, 0.0f, 1.0f);
    if (generateBool)
    {
      if (distance < 0.1f)
      {
        return true;
      }
    }

    wheelMesh.GetVertices(vertices);
    wheelMesh.GetNormals(normals);
    wheelMesh.GetTriangles(tris, 0);
    wheelMesh.GetColors(colors);
    wheelMesh.GetUVs(0, uv);
    int verLenght = GetCurrentVerticeIndexForMesh(wheelMesh);
    int triLength = verLenght * 3;
    vertices[GetRepeatedArrayIndex(vertices, verLenght - 1)] = tempWHeelHit.point + tempWHeelHit.normal * 0.02f - skidTemp + tempWHeelHit.sidewaysDir * lateralDisplacement;
    vertices[GetRepeatedArrayIndex(vertices, verLenght - 2)] = tempWHeelHit.point + tempWHeelHit.normal * 0.02f + skidTemp + tempWHeelHit.sidewaysDir * lateralDisplacement;
    normals[GetRepeatedArrayIndex(normals, verLenght - 1)] = normals[GetRepeatedArrayIndex(normals, verLenght - 2)] = tempWHeelHit.normal;
    Color corRastro = _skidMarks.standardColor;
    corRastro.a = Mathf.Clamp(alphaAplic * 0.9f, 0.01f, 1.0f);
    colors[GetRepeatedArrayIndex(colors, verLenght - 1)] = colors[GetRepeatedArrayIndex(colors, verLenght - 2)] = corRastro;
    tris[GetRepeatedArrayIndex(tris, triLength + 0)] = tris[GetRepeatedArrayIndex(tris, triLength + 3)] =
      tris[GetRepeatedArrayIndex(tris, triLength + 1)] = tris[GetRepeatedArrayIndex(tris, triLength + 4)] =
        tris[GetRepeatedArrayIndex(tris, triLength + 2)] = tris[GetRepeatedArrayIndex(tris, triLength + 5)] =
          tris[GetRepeatedArrayIndex(tris, triLength + 6)] = tris[GetRepeatedArrayIndex(tris, triLength + 9)] =
            tris[GetRepeatedArrayIndex(tris, triLength + 7)] = tris[GetRepeatedArrayIndex(tris, triLength + 10)] =
              tris[GetRepeatedArrayIndex(tris, triLength + 8)] = tris[GetRepeatedArrayIndex(tris, triLength + 11)];
    if (generateBool)
    {
      tris[GetRepeatedArrayIndex(tris, triLength - 1)] = GetRepeatedArrayIndex(vertices, verLenght - 2);
      tris[GetRepeatedArrayIndex(tris, triLength - 2)] = GetRepeatedArrayIndex(vertices, verLenght - 1);
      tris[GetRepeatedArrayIndex(tris, triLength - 3)] = GetRepeatedArrayIndex(vertices, verLenght - 3);
      tris[GetRepeatedArrayIndex(tris, triLength - 4)] = GetRepeatedArrayIndex(vertices, verLenght - 3);
      tris[GetRepeatedArrayIndex(tris, triLength - 5)] = GetRepeatedArrayIndex(vertices, verLenght - 4);
      tris[GetRepeatedArrayIndex(tris, triLength - 6)] = GetRepeatedArrayIndex(vertices, verLenght - 2);
      uv[GetRepeatedArrayIndex(uv, verLenght - 1)] =
        uv[GetRepeatedArrayIndex(uv, verLenght - 3)] + Vector2.right * distance * 0.01f;
      uv[GetRepeatedArrayIndex(uv, verLenght - 2)] =
        uv[GetRepeatedArrayIndex(uv, verLenght - 4)] + Vector2.right * distance * 0.01f;
    }
    else
    {
      uv[GetRepeatedArrayIndex(uv, verLenght - 1)] = Vector2.zero;
      uv[GetRepeatedArrayIndex(uv, verLenght - 2)] = Vector2.up;
    }

    lastPoint[indexLastMark] = vertices[GetRepeatedArrayIndex(vertices, verLenght - 1)];
    wheelMesh.SetVertices(vertices);
    wheelMesh.SetNormals(normals);
    wheelMesh.SetTriangles(tris, 0);
    wheelMesh.SetColors(colors);
    wheelMesh.SetUVs(0, uv);
    wheelMesh.RecalculateBounds();
    return true;
  }

  private Mesh GerarRendRef(Material skdMaterial, string wheelName)
  {
    GameObject rendRef = new GameObject("SkidMesh " + wheelName + " " + transform.name);
    rendRef.AddComponent<MeshFilter>();
    rendRef.AddComponent<MeshRenderer>();
    Mesh mesh = rendRef.GetComponent<MeshFilter>().mesh = new Mesh();
    mesh.vertices = new Vector3[1200];
    mesh.normals = new Vector3[1200];
    mesh.uv = new Vector2[1200];
    mesh.colors = new Color[1200];
    mesh.triangles = new int[3600];
    mesh.MarkDynamic();
    rendRef.GetComponent<MeshRenderer>().material = skdMaterial;
    rendRef.GetComponent<MeshRenderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
    return mesh;
  }

  private void SetSkidMarksValues()
  {
    Material skidmarkMaterial;
    skidmarkMaterial = new Material(skidMarksShader);
    skidmarkMaterial.mainTexture = GenerateTextureAndNormalMap(true);
    skidmarkMaterial.SetTexture("_NormalMap", GenerateTextureAndNormalMap(false));
    skidmarkMaterial.SetFloat("_NormFactor", 0.7f);
    skidmarkMaterial.SetFloat("_Glossiness", 0);
    skidmarkMaterial.SetFloat("_Metallic", 0);
    Color skidColor = _skidMarks.standardColor;
    skidColor.a = 0.9f;
    skidmarkMaterial.color = skidColor;
//
    _wheels.rightFrontWheel.rendSKDmarks = GerarRendRef(skidmarkMaterial, _wheels.rightFrontWheel.wheelCollider.gameObject.transform.name);
    _wheels.leftFrontWheel.rendSKDmarks = GerarRendRef(skidmarkMaterial, _wheels.leftFrontWheel.wheelCollider.gameObject.transform.name);
    _wheels.rightRearWheel.rendSKDmarks = GerarRendRef(skidmarkMaterial, _wheels.rightRearWheel.wheelCollider.gameObject.transform.name);
    _wheels.leftRearWheel.rendSKDmarks = GerarRendRef(skidmarkMaterial, _wheels.leftRearWheel.wheelCollider.gameObject.transform.name);
  }

  public Texture GenerateTextureAndNormalMap(bool isTexture)
  {
    Texture2D texture = new Texture2D(32, 32, TextureFormat.ARGB32, false);
    Color transparentColor1 = new Color(0.0f, 0.0f, 0.0f, 0.5f);
    Color transparentColor2 = new Color(0.0f, 0.0f, 0.0f, 1.0f);
    if (isTexture)
    {
      transparentColor1 = new Color(1.0f, 1.0f, 1.0f, 0.15f);
      transparentColor2 = new Color(1.0f, 1.0f, 1.0f, 0.6f);
    }

    for (int x = 0; x < 32; x++)
    {
      for (int y = 0; y < 32; y++)
      {
        texture.SetPixel(x, y, Color.white);
      }
    }

    for (int y = 0; y < 32; y++)
    {
      for (int x = 0; x < 32; x++)
      {
        if (y == 0 || y == 1 || y == 30 || y == 31)
        {
          texture.SetPixel(x, y, transparentColor1);
        }

        if (y == 6 || y == 7 || y == 15 || y == 16 || y == 24 || y == 25)
        {
          texture.SetPixel(x, y, transparentColor2);
        }
      }
    }

    texture.Apply();
    return texture;
  }

  #endregion
}