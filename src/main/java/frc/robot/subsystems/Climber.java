// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Climber extends SubsystemBase {
  private CANSparkMax leftClimberSparkMax = new CANSparkMax(MechanismConstants.kLeftClimberPort, MotorType.kBrushless);
  private CANSparkMax rightClimberSparkMax = new CANSparkMax(MechanismConstants.kRightClimberPort, MotorType.kBrushless);
  /** Creates a new Climber. */
  public Climber() {
    
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
