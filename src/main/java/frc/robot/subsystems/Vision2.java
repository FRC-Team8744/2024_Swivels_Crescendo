// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision2 extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

Rotation3d rd = new Rotation3d(0, Units.degreesToRadians(-23.7), Units.degreesToRadians(180));
Transform3d td = new Transform3d(-0.04, -0.25, 0, rd);
Transform3d ampOffSet = new Transform3d(0, 0, -0.47, new Rotation3d());
Transform3d speakerOffSet = new Transform3d(0, 0, 0.6, new Rotation3d());
Transform3d stageOffSet = new Transform3d(0, 0, 0.4, new Rotation3d());




  public Vision2() {}


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
    cameraToTarget = cameraToTarget.plus(stageOffSet);
  SmartDashboard.putNumber("Height",135);
    }
  else if (ID == 5 || ID == 6){
  cameraToTarget = cameraToTarget.plus(ampOffSet);
  SmartDashboard.putNumber("Height",61);
  }
  else if (ID == 8 || ID == 7 || ID == 3 || ID == 4){
  cameraToTarget = cameraToTarget.plus(speakerOffSet);
  SmartDashboard.putNumber("Height", 176);
  }
  else SmartDashboard.putNumber("Height", -1);
  Translation3d ampOffSet;

   

      Transform3d targetTd = cameraToTarget.plus(td);

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


  }
    }
