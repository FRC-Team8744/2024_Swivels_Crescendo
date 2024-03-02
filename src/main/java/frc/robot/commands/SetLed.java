// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.LEDS;

public class SetLed extends Command {
  /** Creates a new SetLed. */
  private final LEDS m_led;
  private final Index m_index;
  public SetLed(LEDS le, Index ind) {
    m_led = le;
    m_index = ind;
    addRequirements(m_led);
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.rainbow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_index.inputIR.get() == false) {
      m_led.setRainbow(0, 255, 0);
    }
    else {
      m_led.setRainbow(0, 0, 255);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
