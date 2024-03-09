// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShoot extends SequentialCommandGroup {
  private final Climber m_climber;
  private final Shooter m_shooter;
  private final Index m_index;
  private final LEDS m_led;
  public AmpShoot(Climber cl, Shooter sh, Index ind, LEDS le) {
    m_climber = cl;
    m_shooter = sh;
    m_index = ind;
    m_led = le;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ClimbUp(m_climber).withTimeout(.2),
    new ShootA(m_shooter, m_index, m_led),
    new ClimbDown(m_climber).withTimeout(.2));
  }
}