// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
public class Wing extends SequentialCommandGroup {
  /** Creates a new BlueWing. */
  public Field2d field;
  
  public Wing(DriveSubsystem drive, AutoCommandManager autoCommandManager) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      Translation2d score1 = new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(161.5));

      // Change rotation 2d to 180 if on red
      Trajectory driveToFirstNote = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.blueStart, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.blueWingNote1, new Rotation2d(0))), autoCommandManager.forwardConfig);

      Trajectory driveToFirstScore = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.blueWingNote1, new Rotation2d(0)),
        new Pose2d(score1, new Rotation2d(0))), autoCommandManager.reverseConfig);

      SwerveControllerCommand driveToFirstNoteCommand = autoCommandManager.trajectoryCommand(driveToFirstNote, drive);

      SwerveControllerCommand driveToFirstScoreCommand = autoCommandManager.trajectoryCommand(driveToFirstScore, drive);

      field = new Field2d();
      SmartDashboard.putData(field);
      field.setRobotPose(driveToFirstNote.getInitialPose());
      field.getObject("Drive to first note trajectory").setTrajectory(driveToFirstNote);
      field.getObject("Drive to score trajectory").setTrajectory(driveToFirstScore);
      SmartDashboard.putNumber("Time", driveToFirstNote.getTotalTimeSeconds() + driveToFirstScore.getTotalTimeSeconds());

      SmartDashboard.putString("Auto Alliance", "Blue");

      addCommands(
        new InstantCommand(() -> drive.setEstimatedPose(driveToFirstNote.getInitialPose())),
        new ParallelCommandGroup(driveToFirstNoteCommand, autoCommandManager.m_runIntakeNew),
        driveToFirstScoreCommand,
        autoCommandManager.m_visionShoot
    );
    } else {
      Translation2d score1 = new Translation2d(16.4592 - Units.inchesToMeters(100), Units.inchesToMeters(161.5));

      // Change rotation 2d to 180 if on red
      Trajectory driveToFirstNote = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.redStart, new Rotation2d(Math.PI)),
        new Pose2d(TrajectoryConstants.redWingNote1, new Rotation2d(Math.PI))), autoCommandManager.forwardConfig);

      Trajectory driveToFirstScore = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.redWingNote1, new Rotation2d(Math.PI)),
        new Pose2d(score1, new Rotation2d(Math.PI))), autoCommandManager.reverseConfig);

      SwerveControllerCommand driveToFirstNoteCommand = autoCommandManager.trajectoryCommand(driveToFirstNote, drive);

      SwerveControllerCommand driveToFirstScoreCommand = autoCommandManager.trajectoryCommand(driveToFirstScore, drive);

      field = new Field2d();
      SmartDashboard.putData(field);
      field.setRobotPose(driveToFirstNote.getInitialPose());
      field.getObject("Drive to first note trajectory").setTrajectory(driveToFirstNote);
      field.getObject("Drive to score trajectory").setTrajectory(driveToFirstScore);
      SmartDashboard.putNumber("Time", driveToFirstNote.getTotalTimeSeconds() + driveToFirstScore.getTotalTimeSeconds());

      SmartDashboard.putString("Auto Alliance", "Red");

      addCommands(
        new InstantCommand(() -> drive.setEstimatedPose(driveToFirstNote.getInitialPose())),
        new ParallelCommandGroup(driveToFirstNoteCommand, autoCommandManager.m_runIntakeNew),
        driveToFirstScoreCommand,
        autoCommandManager.m_visionShoot
    );
    }
  }
}
