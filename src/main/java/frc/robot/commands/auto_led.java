// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDS;

public class auto_led extends Command {
  private final LEDS m_lightbarLeds;
  private final DriveSubsystem m_drive;
  private double tv;
  private int x; // x tracks angle.
  private double tx;
  /** Creates a new auto_led. */
  public auto_led(LEDS light, DriveSubsystem drive) {
    m_lightbarLeds = light;
    addRequirements(m_lightbarLeds);
    m_drive = drive;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
m_lightbarLeds.setLed(17,255,255,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tv = SmartDashboard.getNumber("tv", 0);
    tx = SmartDashboard.getNumber("LimelightX", 0);
    x = (int) (-tx * (36.0/30.0) + 17.0);
    if (tv == 1){
      m_lightbarLeds.allOff();
      m_lightbarLeds.setLed(x,1,255,1); //green
    } else {
      m_lightbarLeds.allOff(); // after it's done it will be red
      m_lightbarLeds.setLed(17, 255, 1, 1); // red
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_lightbarLeds.allOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
