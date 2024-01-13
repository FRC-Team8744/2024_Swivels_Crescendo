// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoTurnToAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  /** Creates a new Vison. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");
NetworkTableEntry tid = table.getEntry("tid");
NetworkTableEntry tv = table.getEntry("tv");

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);
double AprilNumber = tid.getDouble(0.0);
double CanSeeTag= tv.getDouble(0.0);

SmartDashboard.putNumber("Tid", AprilNumber);

//post to smart dashboard periodically
if (x > 0) {
  SmartDashboard.putNumber("limelight x positive", x);

}
else if (x <= 0) {
  SmartDashboard.putNumber("limelight x negative", x); 
}
else {
  SmartDashboard.putNumber("limelight x negative", 999);
  SmartDashboard.putNumber("limelight x positive", 999);
 }

if (y > 0) {
  SmartDashboard.putNumber("limelight y positive", y);
}
else if (y <= 0) {
  SmartDashboard.putNumber("limelight y negative", y);
}
else {
  SmartDashboard.putNumber("limelight y negative", 999);
  SmartDashboard.putNumber("limelight y positive", 999);
}

if (x > 5) {
  SmartDashboard.putNumber("limelight x to go", Math.abs(x));
}

if (x < 5) {
  SmartDashboard.putNumber("limelight x to go", Math.abs(x));
}
  }
  }


