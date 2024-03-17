package frc.robot;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Camera{
  UsbCamera camera;
  String cameraName;
  VideoSink server;


  public Camera() {
    setupCameraStream();  
  }


  public void setupCameraStream() {
    // Get the UsbCamera from CameraServer
    camera = CameraServer.startAutomaticCapture("camera", 0);
    cameraName = camera.getName();

    Shuffleboard.getTab("Camera")
                .add("Camera", server.getSource())
                .withWidget(BuiltInWidgets.kCameraStream)
                .withProperties(Map.of("Show controls", false, "Rotation", "QUARTER_CCW"))
                .withSize(5, 5)
                .withPosition(0, 0);
  }
}
