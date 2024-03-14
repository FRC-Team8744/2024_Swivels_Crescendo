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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.CameraTargetRelation;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision2 extends SubsystemBase {
  private PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

  private Rotation3d rd = new Rotation3d(0, Units.degreesToRadians(-23.7), Units.degreesToRadians(180));
  private Transform3d td = new Transform3d(0.04, 0.25, 0, rd);
  private Pose3d targetTd;
  public double distanceToApriltag = 0;
// Transform3d ampOffSet = new Transform3d(0, 0, -0.47, new Rotation3d());
// Transform3d speakerOffSet = new Transform3d(0, 0, 0.6, new Rotation3d());
// Transform3d stageOffSet = new Transform3d(0, 0, 0.4, new Rotation3d());

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

  // Translation2d translation = PhotonUtils.estimateCameratoTargetTranslation(distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));

  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry tx = table.getEntry("tx");
  // NetworkTableEntry ty = table.getEntry("ty");
  // NetworkTableEntry ta = table.getEntry("ta");
  // NetworkTableEntry id = table.getEntry("tid");
  // NetworkTableEntry tv = table.getEntry("tv");
  // NetworkTableEntry tpi = table.getEntry("getpipe");
  // NetworkTableEntry spi = table.getEntry("pipeline");

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
      Transform3d cameraToTarget = localTarget.getBestCameraToTarget();

      Pose3d aprilTagPose3d = aprilTagFieldLayout.getTagPose(localTarget.getFiducialId()).get();

      targetTd = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, aprilTagPose3d, td);

      ID = localTarget.getFiducialId();

      if ((ID == 4) || (ID == 7)) { //blue-7 red-4
        speakerInView = true;
        target = localTarget;
        double angleMultiplier = ID == 7 ? 180 : 180;
        double yaw = (PhotonUtils.getYawToPose(targetTd.toPose2d(), aprilTagPose3d.toPose2d()).getDegrees());
        yaw *= ID == 7 ? 1 : -1;
        distanceToApriltag = PhotonUtils.getDistanceToPose(targetTd.toPose2d(), aprilTagPose3d.toPose2d());
        SmartDashboard.putNumber("yaw", yaw);

        tx_out = yaw; //m_lowpass.calculate(yaw);

        SmartDashboard.putNumber("Filtered Tx", tx_out);


      } else {
        target = null;
        speakerInView = false;
        // m_lowpass.reset();
      }
      speakerInView_filtered = m_filterSpeakerInView.calculate(speakerInView);
      SmartDashboard.putBoolean("SpeakerInView", speakerInView_filtered);


//they have -31 for height of camera
    if (ID >= 11 && ID <= 16){
    //this is chian thingy
      heightMatters = 1.35;
   // cameraToTarget = cameraToTarget.plus(stageOffSet);
  // SmartDashboard.putNumber("Height",135);
    }
  else if (ID == 5 || ID == 6){
    //this is amp
    heightMatters = .61;
  // cameraToTarget = cameraToTarget.plus(ampOffSet);
  // SmartDashboard.putNumber("Height",61);
  }
  else if (ID == 8 || ID == 7 || ID == 3 || ID == 4){
    heightMatters = 1.76;
    //this is speaker
  // cameraToTarget = cameraToTarget.plus(speakerOffSet);
  // SmartDashboard.putNumber("Height", 176);
  }
  else {heightMatters = -1;}
  // Translation3d ampOffSet;
  SmartDashboard.putNumber("Height",heightMatters);

      // Transform3d targetTd = cameraToTarget.plus(td);

      // Pose3d = new Pose3d(robotPose);

      // Pose3d scoringPose3d = pose.plus(targetOffset);


  double yaw = Units.radiansToDegrees(targetTd.getRotation().getZ());
  tx_out = m_lowpass.calculate(yaw);
  SmartDashboard.putNumber("Filtered Tx", tx_out);

  // double pitch = Math.abs(Units.radiansToDegrees( targetTd.getRotation().getY()));
  double pitch = Units.radiansToDegrees(Math.atan(1.76 / targetTd.getX()));
  // double tx = yaw;

  double Apriltagid = ID;
  // SmartDashboard.putNumber("tx", tx);
  SmartDashboard.putNumber("ApriltagIDback", Apriltagid);
  SmartDashboard.putNumber("X", distanceToApriltag);
  SmartDashboard.putNumber("Y distance", targetTd.getY());
  SmartDashboard.putNumber("Y", Units.radiansToDegrees(cameraToTarget.getRotation().getY()));
  // SmartDashboard.putNumber("Ty", targetTd.getY());
  SmartDashboard.putNumber("By", pitch);
} else {
  ID = 0;
}
SmartDashboard.putNumber("Id", ID); 
SmartDashboard.putBoolean("RT", m_debouncer.calculate(result.hasTargets()));
SmartDashboard.putNumber("Angle", tx_out);

// SmartDashboard.putBoolean("isSpeakerInView", isSpeakerInView());

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