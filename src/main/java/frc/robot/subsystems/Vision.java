// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.PipedInputStream;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpiutil.net.PortForwarder;

public class Vision extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry id = table.getEntry("tid");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tpi = table.getEntry("getpipe");
  NetworkTableEntry spi = table.getEntry("pipeline");

  public Vision() {}

  @Override
  public void periodic() {
 //   double tax = LimelightHelpers.getTX("");
 int spi = (0);
double pipeline = tpi.getDouble(0.0);
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);
double AprilNumber = id.getDouble(0.0);
double v = tv.getDouble(0.0);

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