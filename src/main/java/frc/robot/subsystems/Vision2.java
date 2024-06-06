// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision2 extends SubsystemBase {
  private PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

  private Rotation3d rd = new Rotation3d(0, Units.degreesToRadians(-23.7), Units.degreesToRadians(180));
  private Transform3d td = new Transform3d(0.04, 0.25, 0, rd);
  private Pose3d targetTd;
  public double distanceToApriltag = 0;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;

  private double ID = 0;
  private Debouncer m_filterSpeakerInView = new Debouncer (0.1, Debouncer.DebounceType.kBoth );
  private boolean speakerInView;
  private boolean speakerInView_filtered;

  private Debouncer m_debouncer = new Debouncer (0.1, Debouncer.DebounceType.kBoth );
  private LinearFilter m_lowpass = LinearFilter.movingAverage(100);
  private double tx_out;
//**heightMatters is the height of the object based on the april tags and the camera used for cacluations in shooting**//
  private double heightMatters;
  public double m_goalAngle;

  public Vision2() {
    ID = 0;
  }


  @Override
  public void periodic() {
    result = camera.getLatestResult();
    if (result.getMultiTagResult().estimatedPose.isPresent) {
      MultiTargetPNPResult multiTag = result.getMultiTagResult();
    }
    result.getTargets();

    if (result.hasTargets()){
      PhotonTrackedTarget localTarget = result.getBestTarget();

      // Start of check list
      boolean foundSpeaker = false;
      for (var thisTarget : result.targets) {  // Java 'for each' loop
        int myID = thisTarget.getFiducialId();
        if ((myID == 4) || (myID == 7)) {
          foundSpeaker = true;
          localTarget = thisTarget;  // fix 'best' target
        }
        //they have -31 for height of camera
        if (myID >= 11 && myID <= 16){
          //this is chian thingy
          heightMatters = 1.35;
        }
        else if (myID == 5 || myID == 6){
          //this is amp
          heightMatters = .61;
        }
        else if (myID == 8 || myID == 7 || myID == 3 || myID == 4){
          heightMatters = 1.76;
          //this is speaker
        }
        else {heightMatters = -1;}
      }

      Transform3d cameraToTarget = localTarget.getBestCameraToTarget();

      Pose3d aprilTagPose3d = aprilTagFieldLayout.getTagPose(localTarget.getFiducialId()).get();

      targetTd = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, aprilTagPose3d, td);

      ID = localTarget.getFiducialId();

      // if(ID == 3){ID = 4;}

      // if ((ID == 4) || (ID == 7)) { //blue-7 red-4
      if (foundSpeaker) {
        speakerInView = true;
        target = localTarget;
        double angleMultiplier = ID == 7 ? 180 : 180;
        double yaw = (PhotonUtils.getYawToPose(targetTd.toPose2d(), aprilTagPose3d.toPose2d()).getDegrees());
        yaw *= ID == 7 ? 1 : -1;
        distanceToApriltag = PhotonUtils.getDistanceToPose(targetTd.toPose2d(), aprilTagPose3d.toPose2d());
        SmartDashboard.putNumber("yaw", yaw);

        tx_out = yaw; //m_lowpass.calculate(yaw);

        SmartDashboard.putNumber("Filtered Tx", tx_out);
        
        double visionShootAngleTEST = 36;
        if (getTargetDistance() > 8.23) {visionShootAngleTEST = Math.toDegrees(Math.atan(getTargetVertAngle() / Math.abs(getTargetDistance() - 16.459))) + Math.abs(getTargetDistance() - 16.459) * 2/3 -1; }
        else {visionShootAngleTEST = Math.toDegrees(Math.atan(getTargetVertAngle() / getTargetDistance())) + (getTargetDistance() * 2/3 -1);}
        SmartDashboard.putNumber("TEST Vision angle", visionShootAngleTEST);

      } else {
        target = null;
        speakerInView = false;
        // m_lowpass.reset();
      }
      speakerInView_filtered = m_filterSpeakerInView.calculate(speakerInView);
      SmartDashboard.putBoolean("SpeakerInView", speakerInView_filtered);

  SmartDashboard.putNumber("Height",heightMatters);

  double yaw = Units.radiansToDegrees(targetTd.getRotation().getZ());
  tx_out = m_lowpass.calculate(yaw);
  SmartDashboard.putNumber("Filtered Tx", tx_out);

  double pitch = Units.radiansToDegrees(Math.atan(1.76 / targetTd.getX()));

  double Apriltagid = ID;
  SmartDashboard.putNumber("ApriltagIDback", Apriltagid);
  SmartDashboard.putNumber("X", distanceToApriltag);
  SmartDashboard.putNumber("Y distance", targetTd.getY());
  SmartDashboard.putNumber("Y", Units.radiansToDegrees(cameraToTarget.getRotation().getY()));
  SmartDashboard.putNumber("By", pitch);
} else {
  ID = 0;
}
SmartDashboard.putNumber("Id", ID); 
SmartDashboard.putBoolean("RT", m_debouncer.calculate(result.hasTargets()));
SmartDashboard.putNumber("Angle", tx_out);
}
  // port: http://limelight.local:5801/

  public boolean isSpeakerInView() {
    return speakerInView_filtered;
  }

  public double getTargetHorzAngle() {
    return tx_out;
  }

  public double getTargetVertAngle() {
    return heightMatters;
  }

  public double getTargetDistance() {
    return distanceToApriltag;
  }

  public double getTargetYDistance() {
    return targetTd.getY();
  }

  public PhotonTrackedTarget getTarget() {
    return target;
  }
}