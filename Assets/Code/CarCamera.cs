using System.IO;
using UnityEngine;

public class CarCamera : MonoBehaviour
{
  private VehicleController controller;
  private int frameCount;
  private bool started;
  private bool stopped;

  // Use this for initialization
  private void Start()
  {
    controller = FindObjectOfType<VehicleController>();
  }

  private void Update()
  {
    if (controller.started && !started)
    {
      InvokeRepeating(nameof(SaveData), 0, 0.1f);

      started = true;
    }

    if (Input.GetKey(KeyCode.Space))
    {
      stopped = true;
    }
  }

  private void SaveData()
  {
    if (stopped)
    {
      return;
    }

    string frame = $"frame{frameCount:D2}";

    ScreenCapture.CaptureScreenshot(Path.Combine(Directory.GetCurrentDirectory(), $"Images\\Straight-{frame}.png"));
    File.WriteAllText(Path.Combine(Directory.GetCurrentDirectory(), $"Images\\Straight-{frame}.data"), $"{controller.verticalInput}, {controller.horizontalInput}");

    frameCount++;
  }
}