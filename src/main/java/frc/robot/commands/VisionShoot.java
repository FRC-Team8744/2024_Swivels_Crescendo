// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2;

public class VisionShoot extends Command {
  private final Shooter m_shooter;
  private final Index m_index;
  private final LEDS m_led;
  private final Vision2 m_vis;
  private final Pivot m_pivot;
  private final Timer m_timer = new Timer();
  private int sensorState = 0;

  public VisionShoot(Shooter sh, Index ind, LEDS le, Vision2 vi, Pivot pi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = sh;
    m_index = ind;
    m_led = le;
    m_vis = vi;
    m_pivot = pi;
    addRequirements(m_shooter);
    addRequirements(m_index);
    addRequirements(m_led);
    addRequirements(m_vis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensorState = 0;
    m_shooter.testShoot(m_shooter.visionShootVelocity);
    m_pivot.testAngle(m_pivot.visionShootAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_pivot.visionShootAngle == 26) {
      m_led.ledOn(255, 0, 0);
    }
    else {
      m_led.rainbow();
    }
    if (m_shooter.visionAtSpeed()) {
      m_index.indexRun(-m_index.indexSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_index.indexStop();
    m_pivot.stopAngle();
    m_led.ledOn(0, 0, 128);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Sensor State", sensorState);
    if ((sensorState == 0) && (m_index.inputIR.get() == false)) sensorState = 1;
    if ((sensorState == 1) && (m_index.inputIR.get() == true)) {
    sensorState = 2; 
    m_timer.restart();}
    if ((sensorState == 2) && (m_timer.get() >= 0.1)) return true;
    return false;
  }
}