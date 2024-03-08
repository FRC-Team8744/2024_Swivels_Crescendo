// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSpinUp extends SequentialCommandGroup {
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Index m_index;
  private final LEDS m_leds;
  public IntakeSpinUp(Intake in, Shooter sh, Index ind, LEDS le) {
    m_intake = in;
    m_shooter = sh;
    m_index = ind;
    m_leds = le;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakeRun(m_intake, m_shooter, m_index, m_leds),
    new RevShooter(m_shooter, m_index).withTimeout(10));
  }
}
