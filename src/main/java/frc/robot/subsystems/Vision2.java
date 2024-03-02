// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
// import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.PipedInputStream;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.CameraTargetRelation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision2 extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

Rotation3d rd = new Rotation3d(0, Units.degreesToRadians(-23.7), Units.degreesToRadians(180));
Transform3d td = new Transform3d(-0.04, 0.25, 0, rd);
// Transform3d ampOffSet = new Transform3d(0, 0, -0.47, new Rotation3d());
// Transform3d speakerOffSet = new Transform3d(0, 0, 0.6, new Rotation3d());
// Transform3d stageOffSet = new Transform3d(0, 0, 0.4, new Rotation3d());

AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


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

    // SmartDashboard.putNumber("speaker", speakerOffSet.getZ());
  PhotonPipelineResult result = camera.getLatestResult();
  // result.hasTargets();
  PhotonTrackedTarget target = result.getBestTarget();
if (result.hasTargets()){
    Transform3d cameraToTarget = target.getBestCameraToTarget();
    double ID = target.getFiducialId();
    
    if (ID >= 11 && ID <= 16){
    // cameraToTarget = cameraToTarget.plus(stageOffSet);
  SmartDashboard.putNumber("Height",135);
    }
  else if (ID == 5 || ID == 6){
  // cameraToTarget = cameraToTarget.plus(ampOffSet);
  SmartDashboard.putNumber("Height",61);
  }
  else if (ID == 8 || ID == 7 || ID == 3 || ID == 4){
  // cameraToTarget = cameraToTarget.plus(speakerOffSet);
  SmartDashboard.putNumber("Height", 176);
  }
  else SmartDashboard.putNumber("Height", -1);
  // Translation3d ampOffSet;

   Pose3d targetTd = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), td);

      // Transform3d targetTd = cameraToTarget.plus(td);

      // Pose3d = new Pose3d(robotPose);

      // Pose3d scoringPose3d = pose.plus(targetOffset);


  double yaw = Units.radiansToDegrees( targetTd.getRotation().getZ());
  // double pitch = Math.abs(Units.radiansToDegrees( targetTd.getRotation().getY()));
  double pitch = Units.radiansToDegrees(Math.atan(targetTd.getZ() / targetTd.getX()));
  double tx = yaw;

  double Apriltagid = ID;
  SmartDashboard.putNumber("tx", tx);
  SmartDashboard.putNumber("ApriltagIDback", Apriltagid);

  SmartDashboard.putNumber("TTd", targetTd.getX());
  SmartDashboard.putNumber("CT", Units.radiansToDegrees(cameraToTarget.getRotation().getY()));
  // SmartDashboard.putNumber("Tz", targetTd.getZ());
  // SmartDashboard.putNumber("Ty", targetTd.getY());
  SmartDashboard.putNumber("By", pitch);
  

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
