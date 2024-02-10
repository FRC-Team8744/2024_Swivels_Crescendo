package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class tesst extends Command  {
    public tesst (Boolean boo){this.boo=boo;}
    private Boolean boo; 
    @Override
  public void execute() {
    SmartDashboard.putBoolean("TestBoo", boo);
    boo=!boo;}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
