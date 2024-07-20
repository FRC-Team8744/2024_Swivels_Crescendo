// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;

public class SourceIntake extends Command {
  private final Shooter m_shooter;
  private final Index m_index;
  private final LEDS m_led;
  private int sensorState = 0;
  /** Creates a new UnDonut. */
  public SourceIntake(Shooter sh, Index ind, LEDS le) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = sh;
    m_index = ind;
    m_led = le;
    addRequirements(m_shooter);
    addRequirements(m_index);
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.testShoot(-1500);
    m_shooter.m_pivot.testAngle(60);
    m_index.indexRun(m_index.indexSpeed);
    m_led.setSlashLed(128, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.m_pivot.stopAngle();
    m_shooter.stopShooter();
    m_index.indexStop();
    if (m_index.inputIR.get() == false) {
      m_led.setSlashLed(0, 128, 0);
    }
    else {
      m_led.setSlashLed(0, 0, 128);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((sensorState == 0) && (m_index.inputIR.get() == true)) sensorState = 1;
    if ((sensorState == 1) && (m_index.inputIR.get() == false)) sensorState = 2;
    if ((sensorState == 2) && (m_index.inputIR.get() == true)) sensorState = 3;
    if ((sensorState == 3) && (m_index.inputIR.get() == false)) return true;
    return false;
  }
}
