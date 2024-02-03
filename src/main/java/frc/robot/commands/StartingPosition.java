// Copyright (c) FIRST and other WPILib contribTOtors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class StartingPosition extends Command {
  private final DriveSubsystem m_drive;
  /** Creates a new StartingPosition. */
  public StartingPosition(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(new Pose2d(new Translation2d(1.21, 4.12), new Rotation2d(0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
