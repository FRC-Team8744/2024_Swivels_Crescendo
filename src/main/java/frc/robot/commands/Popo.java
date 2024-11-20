// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.motorspin;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Popo extends SequentialCommandGroup {
    private final Climber m_climber;
    private final motorspin m_mymotor;

  /** Creates a new Popo. */
  public Popo(Climber cl, motorspin ms) {
    m_climber = cl;
    m_mymotor = ms; 

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new motorspin_comand(m_mymotor).withTimeout(.5),
    new ClimbUp(m_climber).withTimeout(0.2));
  }
}
