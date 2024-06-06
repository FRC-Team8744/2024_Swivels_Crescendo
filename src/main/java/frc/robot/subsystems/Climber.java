// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Climber extends SubsystemBase {
  private CANSparkMax leftClimberSparkMax = new CANSparkMax(MechanismConstants.kLeftClimberPort, MotorType.kBrushless);
  private CANSparkMax rightClimberSparkMax = new CANSparkMax(MechanismConstants.kRightClimberPort, MotorType.kBrushless);

  private final RelativeEncoder leftClimberEncoder = leftClimberSparkMax.getEncoder();

  public double climbSpeed = .6;
  /** Creates a new Climber. */
  public Climber() {
    rightClimberSparkMax.follow(leftClimberSparkMax, true);
 
    leftClimberSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightClimberSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void climberUp(double speed) {
    leftClimberSparkMax.setSmartCurrentLimit(10);
    rightClimberSparkMax.setSmartCurrentLimit(10);
    leftClimberSparkMax.set(speed);
  }

  public void climberDown(double speed) {
    leftClimberSparkMax.setSmartCurrentLimit(40);
    rightClimberSparkMax.setSmartCurrentLimit(40);
    leftClimberSparkMax.set(-speed);
  }

  public void stopClimber() {
    leftClimberSparkMax.stopMotor();
    rightClimberSparkMax.stopMotor();
  }

  public double climberVelocity() {
    return leftClimberEncoder.getVelocity();
  }

  @Override
  public void periodic() {

  }
}