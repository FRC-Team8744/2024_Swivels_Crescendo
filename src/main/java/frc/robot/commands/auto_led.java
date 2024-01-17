// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Vision; // Import the Vision subsystem

public class auto_led extends Command {
    private final LEDS m_ledstrip;
    private final Vision m_vision; // Add Vision subsystem

    /** Creates a new auto_led. */
    public auto_led(LEDS ledstrip, Vision vision) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_ledstrip = ledstrip;
        m_vision = vision; // Initialize Vision subsystem
        addRequirements(m_ledstrip, m_vision); // Add both subsystems as requirements
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Limelight aiming logic
        double x = m_vision.getTx(); // Get horizontal angle from Limelight
        double canSeeTarget = m_vision.getTv(); // Check if Limelight sees a target

        if (canSeeTarget == 1.0) {
            // Aiming logic - adjust LED strip or perform actions based on your requirements
            double steeringAdjust = 0.1 * x; // Adjust this value as needed

            // Perform actions based on the aiming logic
            // Example: Adjust LED strip based on the steering adjustment
            m_ledstrip.adjustLEDs(steeringAdjust);
        } else {
            // No target detected, provide feedback or take appropriate actions
            m_ledstrip.defaultLEDs(); // Reset LED strip to default state
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Optionally reset or perform cleanup actions when the command ends
        m_ledstrip.defaultLEDs(); // Reset LED strip to default state
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
