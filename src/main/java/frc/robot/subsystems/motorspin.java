package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.MechanismConstants;

public class motorspin extends SubsystemBase {

public double intakeSpeed = 0.7;
  private CANSparkMax frontIntakeSparkMax = new CANSparkMax(MechanismConstants.kFrontIntakePort, MotorType.kBrushless);
  public motorspin() {
    frontIntakeSparkMax.setIdleMode(IdleMode.kBrake);
    frontIntakeSparkMax.setInverted(true);
  }

  public void motorStart(double speed) {
    frontIntakeSparkMax.set(speed);  
  }
 
    public void motorReverse(double speed) {
    frontIntakeSparkMax.set(-speed); 
  }

  public void motorOff() {
    frontIntakeSparkMax.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
