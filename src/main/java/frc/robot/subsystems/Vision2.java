// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.PipedInputStream;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision2 extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

  // Translation2d translation = PhotonUtils.estimateCameratoTargetTranslation(distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));

  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry tx = table.getEntry("tx");
  // NetworkTableEntry ty = table.getEntry("ty");
  // NetworkTableEntry ta = table.getEntry("ta");
  // NetworkTableEntry id = table.getEntry("tid");
  // NetworkTableEntry tv = table.getEntry("tv");
  // NetworkTableEntry tpi = table.getEntry("getpipe");
  // NetworkTableEntry spi = table.getEntry("pipeline");

  public Vision2() {}

//  @Override
//    public void robotInit()
//     for (int port = 5800; port <= 5807; port++) {
// PortForwarder.add(port, "http://photonvision.local:5800/", port);
//    };

  @Override
  public void periodic() {

  PhotonPipelineResult result = camera.getLatestResult();
  // result.hasTargets();
  PhotonTrackedTarget target = result.getBestTarget();
if (result.hasTargets()){
  double yaw = target.getYaw();
  double tx = yaw;
  double ID = target.getFiducialId();
  double Apriltagid = ID;
  SmartDashboard.putNumber("tx", tx);
  SmartDashboard.putNumber("ApriltagIDback", Apriltagid);
}
 
SmartDashboard.putBoolean("RT", result.hasTargets());


//  //   double tax = LimelightHelpers.getTX("");
//  int spi = (0);
// double pipeline = tpi.getDouble(0.0);
// double x = tx.getDouble(0.0);
// double y = ty.getDouble(0.0);
// double area = ta.getDouble(0.0);
// double AprilNumber = id.getDouble(0.0);
// double v = tv.getDouble(0.0);

//post to smart dashboard periodically

// SmartDashboard.putNumber("LimelightX", x);
// SmartDashboard.putNumber("LimelightY", y);
// SmartDashboard.putNumber("LimelightArea", area);
// SmartDashboard.putNumber("id", AprilNumber);
// SmartDashboard.putNumber("tv", v);
// SmartDashboard.putNumber("pipeline", pipeline);
// SmartDashboard.putNumber("Pipe", spi);
// This method will be called once per scheduler run
  // }
  // public void setPipeline(int piepline){
  //   NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
  //   pipelineEntry.setNumber(pipeline);
  }
  //  @Override
  //   public void robotInit()
  //   {
  //       // Make sure you only configure port forwarding once in your robot code.
  //       // Do not place these function calls in any periodic functions
  //       for (int port = 5800; port <= 5807; port++) {
  //           PortForwarder.add(port, "http://limelight.local:5801/", port);
  //       }
    }
  // }
