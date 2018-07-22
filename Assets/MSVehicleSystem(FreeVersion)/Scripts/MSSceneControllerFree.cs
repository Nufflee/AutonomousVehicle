﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System.Linq;
using System;

[Serializable]
public class ControlsFree
{
  [Space(10)] [Tooltip("If this variable is true, the control for this variable will be activated.")]
  public bool enable_reloadScene_Input = true;

  [Tooltip("The key that must be pressed to reload the current scene.")]
  public KeyCode reloadScene = KeyCode.R;

  [Space(10)] [Tooltip("If this variable is true, the control for this variable will be activated.")]
  public bool enable_startTheVehicle_Input = true;

  [Tooltip("The key that must be pressed to turn the vehicle engine on or off.")]
  public KeyCode startTheVehicle = KeyCode.F;

  [Space(10)] [Tooltip("If this variable is true, the control for this variable will be activated.")]
  public bool enable_enterEndExit_Input = true;

  [Tooltip("The key that must be pressed to enter or exit the vehicle.")]
  public KeyCode enterEndExit = KeyCode.T;

  [Space(10)] [Tooltip("If this variable is true, the control for this variable will be activated.")]
  public bool enable_handBrakeInput_Input = true;

  [Tooltip("The key that must be pressed to activate or deactivate the vehicle hand brake.")]
  public KeyCode handBrakeInput = KeyCode.Space;

  [Space(10)] [Tooltip("If this variable is true, the control for this variable will be activated.")]
  public bool enable_switchingCameras_Input = true;

  [Tooltip("The key that must be pressed to toggle between the cameras of the vehicle.")]
  public KeyCode switchingCameras = KeyCode.C;

  [Space(10)] [Tooltip("If this variable is true, the control for this variable will be activated.")]
  public bool enable_pause_Input = true;

  [Tooltip("The key that must be pressed to pause the game.")]
  public KeyCode pause = KeyCode.P;
}

public class MSSceneControllerFree : MonoBehaviour
{
  #region defineInputs

  [Tooltip("Vertical input recognized by the system")]
  public string _verticalInput = "Vertical";

  [Tooltip("Horizontal input recognized by the system")]
  public string _horizontalInput = "Horizontal";

  [Tooltip("Horizontal input for camera movements")]
  public string _mouseXInput = "Mouse X";

  [Tooltip("Vertical input for camera movements")]
  public string _mouseYInput = "Mouse Y";

  [Tooltip("Scroll input, to zoom in and out of the cameras.")]
  public string _mouseScrollWheelInput = "Mouse ScrollWheel";

  #endregion

  public enum ControlTypeFree
  {
    windows,
    mobileButton
  };

  [Space(10)] [Tooltip("Here you can configure the vehicle controls, choose the desired inputs and also, deactivate the unwanted ones.")]
  public ControlsFree controls;

  [Tooltip("All vehicles in the scene containing the 'MS Vehicle Controller' component must be associated with this list.")]
  public GameObject[] vehicles;

  [Space(10)] [Tooltip("This variable is responsible for defining the vehicle in which the player will start. It represents an index of the 'vehicles' list, where the number placed here represents the index of the list. The selected index will be the starting vehicle.")]
  public int startingVehicle = 0;

  [Tooltip("The player must be associated with this variable. This variable should only be used if your scene also has a player other than a vehicle. This \"player\" will temporarily be disabled when you get in a vehicle, and will be activated again when you leave a vehicle.")]
  public GameObject player;

  [Tooltip("If this variable is true and if you have a player associated with the 'player' variable, the game will start at the player. Otherwise, the game will start in the starting vehicle, selected in the variable 'startingVehicle'.")]
  public bool startInPlayer = false;

  [Tooltip("Here you can select the type of control, where 'Mobile Button' will cause buttons to appear on the screen so that vehicles can be controlled, 'Mobile Joystick' will cause two Joysticks to appear on the screen so vehicles can be controlled, And 'windows' will allow vehicles to be controlled through the computer.")]
  public ControlTypeFree selectControls = ControlTypeFree.windows;

  [Tooltip("This is the minimum distance the player needs to be in relation to the door of a vehicle, to interact with it.")]
  public float minDistance = 3;

  [Space(10)] [Tooltip("If this variable is true, useful data will appear on the screen, such as the car's current gear, speed, brakes, among other things.")]
  public bool UIVisualizer = true;

  private Button cameraMobileButton;

  private Button enterAndExitButton;

  //
  private Button nextVehicle;
  private Button previousVehicle;
  private Text gearText;
  private Text kmhText;
  private Text mphText;
  private Text handBrakeText;
  private Text pauseText;
  private Image backGround;

  #region customizeInputs

  [HideInInspector] public float verticalInput = 0;
  [HideInInspector] public float horizontalInput = 0;
  [HideInInspector] public float mouseXInput = 0;
  [HideInInspector] public float mouseYInput = 0;
  [HideInInspector] public float mouseScrollWheelInput = 0;

  #endregion

  private int currentVehicle = 0;
  private int clampGear;
  private int proximityObjectIndex;
  private int proximityDoorIndex;
  private bool blockedInteraction = false;
  private bool pause = false;
  private bool error;
  private bool enterAndExitBool;
  private string sceneName;

  private VehicleController vehicleCode;
  private VehicleController controllerTemp;
  private float currentDistanceTemp;
  private float proximityDistanceTemp;

  private float MSbuttonHorizontal;
  private float MSbuttonVertical;

  private bool playerIsNull;

  private Vector2 vectorDirJoystick;

  private void Awake()
  {
    error = false;
    CheckEqualKeyCodes();
    MSSceneControllerFree[] sceneControllers = FindObjectsOfType(typeof(MSSceneControllerFree)) as MSSceneControllerFree[];
    if (sceneControllers.Length > 1)
    {
      Debug.LogError("Only one controller is allowed per scene, otherwise the controllers would conflict with each other.");
      error = true;
      for (int x = 0; x < sceneControllers.Length; x++)
      {
        sceneControllers[x].gameObject.SetActive(false);
      }
    }

    if (startingVehicle >= vehicles.Length)
    {
      error = true;
      Debug.LogError("Vehicle selected to start does not exist in the 'vehicles' list");
    }

    if (vehicles.Length == 0)
    {
      error = true;
      Debug.LogError("There is no vehicle in the scene or no vehicle has been associated with the controller.");
    }

    for (int x = 0; x < vehicles.Length; x++)
    {
      if (vehicles[x])
      {
        if (!vehicles[x].GetComponent<VehicleController>())
        {
          error = true;
          Debug.LogError("The vehicle associated with the index " + x + " does not have the 'MSVehicleController' component. So it will be disabled.");
        }
      }
      else
      {
        error = true;
        Debug.LogError("No vehicle was associated with the index " + x + " of the vehicle list.");
      }
    }

    gearText = transform.Find("Canvas/Strings/gearText").GetComponent<Text>();
    kmhText = transform.Find("Canvas/Strings/kmhText").GetComponent<Text>();
    mphText = transform.Find("Canvas/Strings/mphText").GetComponent<Text>();
    handBrakeText = transform.Find("Canvas/Strings/handBrakeText").GetComponent<Text>();
    pauseText = transform.Find("Canvas/Strings/pauseText").GetComponent<Text>();
    backGround = transform.Find("Canvas/Strings").GetComponent<Image>();
    //end transform.find


    vehicleCode = vehicles[currentVehicle].GetComponent<VehicleController>();
    EnableOrDisableButtons(vehicleCode.isInsideTheCar);

    Time.timeScale = 1;
    enterAndExitBool = false;
    sceneName = SceneManager.GetActiveScene().name;
    currentVehicle = startingVehicle;
    //
    for (int x = 0; x < vehicles.Length; x++)
    {
      if (vehicles[x])
      {
        vehicles[x].GetComponent<VehicleController>().isInsideTheCar = false;
      }
    }

    playerIsNull = false;
    if (player)
    {
      player.SetActive(false);
    }
    else
    {
      playerIsNull = true;
    }

    if (startInPlayer)
    {
      if (player)
      {
        player.SetActive(true);
      }
      else
      {
        startInPlayer = false;
        if (vehicles.Length > startingVehicle && vehicles[currentVehicle])
        {
          vehicles[startingVehicle].GetComponent<VehicleController>().isInsideTheCar = true;
        }
      }
    }
    else
    {
      if (vehicles.Length > startingVehicle && vehicles[currentVehicle])
      {
        vehicles[startingVehicle].GetComponent<VehicleController>().isInsideTheCar = true;
      }
    }
  }

  private void CheckEqualKeyCodes()
  {
    var type = typeof(ControlsFree);
    var fields = type.GetFields();
    var values = (from field in fields
      where field.FieldType == typeof(KeyCode)
      select (KeyCode) field.GetValue(controls)).ToArray();

    foreach (var value in values)
    {
      if (Array.FindAll(values, (a) => { return a == value; }).Length > 1)
      {
        Debug.LogError("There are similar commands in the 'controls' list. Use different keys for each command.");
        error = true;
      }
    }
  }

  private void Update()
  {
    if (!error)
    {
      #region customizeInputsValues

      switch (selectControls)
      {
        case ControlTypeFree.windows:
          verticalInput = Input.GetAxis(_verticalInput);
          horizontalInput = Input.GetAxis(_horizontalInput);
          mouseXInput = Input.GetAxis(_mouseXInput);
          mouseYInput = Input.GetAxis(_mouseYInput);
          mouseScrollWheelInput = Input.GetAxis(_mouseScrollWheelInput);
          break;
      }

      #endregion

      vehicleCode = vehicles[currentVehicle].GetComponent<VehicleController>();
      EnableOrDisableButtons(vehicleCode.isInsideTheCar);

      if (Input.GetKeyDown(controls.reloadScene) && controls.enable_reloadScene_Input)
      {
        SceneManager.LoadScene(sceneName);
      }

      if (Input.GetKeyDown(controls.pause) && controls.enable_pause_Input)
      {
        pause = !pause;
      }

      if (pause)
      {
        Time.timeScale = Mathf.Lerp(Time.timeScale, 0.0f, Time.fixedDeltaTime * 5.0f);
      }
      else
      {
        Time.timeScale = Mathf.Lerp(Time.timeScale, 1.0f, Time.fixedDeltaTime * 5.0f);
      }

      if ((Input.GetKeyDown(controls.enterEndExit) || enterAndExitBool) && !blockedInteraction && player && controls.enable_enterEndExit_Input)
      {
        if (vehicles.Length <= 1)
        {
          if (vehicleCode.isInsideTheCar)
          {
            vehicleCode.ExitTheVehicle();
            if (player)
            {
              player.SetActive(true);
              if (vehicleCode.doorPosition[0].transform.position != vehicles[currentVehicle].transform.position)
              {
                player.transform.position = vehicleCode.doorPosition[0].transform.position;
              }
              else
              {
                player.transform.position = vehicleCode.doorPosition[0].transform.position + Vector3.up * 3.0f;
              }
            }

            blockedInteraction = true;
            enterAndExitBool = false;
            StartCoroutine(nameof(WaitToInteract));
          }
          else
          {
            currentDistanceTemp = Vector3.Distance(player.transform.position, vehicleCode.doorPosition[0].transform.position);
            for (int x = 0; x < vehicleCode.doorPosition.Length; x++)
            {
              proximityDistanceTemp = Vector3.Distance(player.transform.position, vehicleCode.doorPosition[x].transform.position);
              if (proximityDistanceTemp < currentDistanceTemp)
              {
                currentDistanceTemp = proximityDistanceTemp;
              }
            }

            if (currentDistanceTemp < minDistance)
            {
              vehicleCode.EnterInVehicle();
              if (player)
              {
                player.SetActive(false);
              }

              blockedInteraction = true;
              enterAndExitBool = false;
              StartCoroutine(nameof(WaitToInteract));
            }
            else
            {
              enterAndExitBool = false;
            }
          }
        }
        else
        {
          proximityObjectIndex = 0;
          proximityDoorIndex = 0;
          for (int x = 0; x < vehicles.Length; x++)
          {
            controllerTemp = vehicles[x].GetComponent<VehicleController>();

            for (int y = 0; y < controllerTemp.doorPosition.Length; y++)
            {
              currentDistanceTemp = Vector3.Distance(player.transform.position, controllerTemp.doorPosition[y].transform.position);
              proximityDistanceTemp = Vector3.Distance(player.transform.position, vehicles[proximityObjectIndex].GetComponent<VehicleController>().doorPosition[proximityDoorIndex].transform.position);
              if (currentDistanceTemp < proximityDistanceTemp)
              {
                proximityObjectIndex = x;
                proximityDoorIndex = y;
              }
            }
          }

          if (vehicleCode.isInsideTheCar)
          {
            vehicleCode.ExitTheVehicle();
            if (player)
            {
              player.SetActive(true);
              if (vehicleCode.doorPosition[0].transform.position != vehicles[currentVehicle].transform.position)
              {
                player.transform.position = vehicleCode.doorPosition[0].transform.position;
              }
              else
              {
                player.transform.position = vehicleCode.doorPosition[0].transform.position + Vector3.up * 3.0f;
              }
            }

            blockedInteraction = true;
            enterAndExitBool = false;
            StartCoroutine(nameof(WaitToInteract));
          }
          else
          {
            controllerTemp = vehicles[proximityObjectIndex].GetComponent<VehicleController>();
            proximityDistanceTemp = Vector3.Distance(player.transform.position, controllerTemp.doorPosition[0].transform.position);
            for (int x = 0; x < controllerTemp.doorPosition.Length; x++)
            {
              currentDistanceTemp = Vector3.Distance(player.transform.position, controllerTemp.doorPosition[x].transform.position);
              if (currentDistanceTemp < proximityDistanceTemp)
              {
                proximityDistanceTemp = currentDistanceTemp;
              }
            }

            if (proximityDistanceTemp < minDistance)
            {
              currentVehicle = proximityObjectIndex;
              vehicles[currentVehicle].GetComponent<VehicleController>().EnterInVehicle();
              if (player)
              {
                player.SetActive(false);
              }

              blockedInteraction = true;
              enterAndExitBool = false;
              StartCoroutine(nameof(WaitToInteract));
            }
            else
            {
              enterAndExitBool = false;
            }
          }
        }
      }

      //
      if (vehicles.Length > 0 && currentVehicle < vehicles.Length && UIVisualizer && vehicleCode)
      {
        if (vehicleCode.isInsideTheCar)
        {
          clampGear = Mathf.Clamp(vehicleCode.currentGear, -1, 1);
          if (clampGear == 0)
          {
            clampGear = 1;
          }

          gearText.text = "Gear: " + vehicleCode.currentGear;
          kmhText.text = "Velocity(km/h): " + (int) (vehicleCode.KMh * clampGear);
          mphText.text = "Velocity(mp/h): " + (int) (vehicleCode.KMh * 0.621371f * clampGear);
          handBrakeText.text = "HandBreak: " + vehicleCode.handBrakeTrue;
          pauseText.text = "Pause: " + pause;
        }
      }
    }
  }

  public void PreviousVehicle()
  {
    if (playerIsNull)
    {
      if (vehicles.Length > 1)
      {
        currentVehicle--;
        EnableVehicle(currentVehicle + 1);
      }
    }
    else
    {
      if (vehicles.Length > 1 && !player.gameObject.activeInHierarchy)
      {
        currentVehicle--;
        EnableVehicle(currentVehicle + 1);
      }
    }
  }

  public void NextVehicle()
  {
    if (playerIsNull)
    {
      if (vehicles.Length > 1)
      {
        currentVehicle++;
        EnableVehicle(currentVehicle - 1);
      }
    }
    else
    {
      if (vehicles.Length > 1 && !player.gameObject.activeInHierarchy)
      {
        currentVehicle++;
        EnableVehicle(currentVehicle - 1);
      }
    }
  }

  private IEnumerator WaitToInteract()
  {
    yield return new WaitForSeconds(0.7f);
    blockedInteraction = false;
  }

  private void EnableOrDisableButtons(bool insideInCar)
  {
    switch (selectControls)
    {
      case ControlTypeFree.mobileButton:
        //enter and exit
        if (enterAndExitButton)
        {
          enterAndExitButton.gameObject.SetActive(true);
        }

        //camera switch e joystick camera
        if (cameraMobileButton)
        {
          cameraMobileButton.gameObject.SetActive(insideInCar);
        }

        break;
      case ControlTypeFree.windows:
        if (cameraMobileButton)
        {
          cameraMobileButton.gameObject.SetActive(false);
        }

        if (enterAndExitButton)
        {
          enterAndExitButton.gameObject.SetActive(false);
        }

        break;
    }
  }

  private void EnableVehicle(int index)
  {
    currentVehicle = Mathf.Clamp(currentVehicle, 0, vehicles.Length - 1);
    if (index != currentVehicle)
    {
      if (vehicles[currentVehicle])
      {
        //change vehicle
        for (int x = 0; x < vehicles.Length; x++)
        {
          vehicles[x].GetComponent<VehicleController>().ExitTheVehicle();
        }

        vehicles[currentVehicle].GetComponent<VehicleController>().EnterInVehicle();
        vehicleCode = vehicles[currentVehicle].GetComponent<VehicleController>();
      }
    }
  }

  private void Mobile_EnterAndExitVehicle()
  {
    if (!error && !enterAndExitBool)
    {
      enterAndExitBool = true;
    }
  }
}