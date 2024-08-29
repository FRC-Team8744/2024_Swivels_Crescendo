// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.TrajectoryAutoPaths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.AutoCommandManager;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueWing extends SequentialCommandGroup {
  /** Creates a new BlueWing. */
  public Field2d field;
  
  public BlueWing(DriveSubsystem drive, AutoCommandManager autoCommandManager) {
    Translation2d score1 = new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(161.5));

    // Change rotation 2d to 180 if on red
    Trajectory driveToFirstNote = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(TrajectoryConstants.start, new Rotation2d(0)),
      new Pose2d(TrajectoryConstants.blueWingNote1, new Rotation2d(0))), autoCommandManager.forwardConfig);

    Trajectory driveToFirstScore = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(TrajectoryConstants.blueWingNote1, new Rotation2d(0)),
      new Pose2d(score1, new Rotation2d(0))), autoCommandManager.reverseConfig);

    SwerveControllerCommand driveToFirstNoteCommand = autoCommandManager.trajectoryCommand(driveToFirstNote, drive);

    SwerveControllerCommand driveToFirstScoreCommand = autoCommandManager.trajectoryCommand(driveToFirstScore, drive);

    // TODO: Move simulation code somewhere else
    field = new Field2d();

    SmartDashboard.putData(field);

    field.setRobotPose(driveToFirstNote.getInitialPose());
    
    field.getObject("Drive to first note trajectory").setTrajectory(driveToFirstNote);
    field.getObject("Drive to score trajectory").setTrajectory(driveToFirstScore);

    System.out.println("TIME: " + driveToFirstNote.getTotalTimeSeconds() + driveToFirstScore.getTotalTimeSeconds());

    addCommands(
      new InstantCommand(() -> drive.setEstimatedPose(driveToFirstNote.getInitialPose())),
      new ParallelCommandGroup(driveToFirstNoteCommand, autoCommandManager.m_runIntakeNew),
      driveToFirstScoreCommand,
      autoCommandManager.m_visionShoot
    );
  }
}
