// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Vision2;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSpinUpTeleop extends SequentialCommandGroup {
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Index m_index;
  private final LEDS m_leds;
  private final Pivot m_pivot;
  private final Vision2 m_vision2;
  private final DriveSubsystem m_driveSubsystem;
  private final XboxController xboxController;
  private final LockOnShooter lockOnShooter;
  public IntakeSpinUpTeleop(Intake in, Shooter sh, Index ind, LEDS le, Pivot pi, Vision2 vis, DriveSubsystem dr, XboxController xb, LockOnShooter ls) {
    m_intake = in;
    m_shooter = sh;
    m_index = ind;
    m_leds = le;
    m_pivot = pi;
    m_vision2 = vis;
    m_driveSubsystem = dr;
    xboxController = xb;
    lockOnShooter = ls;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new IntakeRun(m_intake, m_shooter, m_index, m_leds), (Commands.runOnce(() -> xboxController.setRumble(RumbleType.kBothRumble, 1)))).finallyDo(() -> xboxController.setRumble(RumbleType.kBothRumble, 0)),
    new ParallelCommandGroup(Commands.runOnce(() -> lockOnShooter.toggle()), Commands.runOnce (() -> m_driveSubsystem.isAutoRotate = !m_driveSubsystem.isAutoRotate)));
  }
}
