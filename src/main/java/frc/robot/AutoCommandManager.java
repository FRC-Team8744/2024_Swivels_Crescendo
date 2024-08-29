// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeSpinUp;
import frc.robot.commands.IntakeSpinUpAuto;
import frc.robot.commands.ShootRing;
import frc.robot.commands.VisionShoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.LockOnShooterAuto;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2;
import frc.robot.TrajectoryAutoPaths.BlueWing;

/** Add your docs here. */
public class AutoCommandManager {
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public final LockOnShooterAuto m_lockAuto;
    public HolonomicDriveController holonomicDriveController;
    public IntakeSpinUpAuto m_runIntakeNew;
    public WrapperCommand m_visionShoot;

    public TrajectoryConfig forwardConfig;
    public TrajectoryConfig reverseConfig;

    public AutoCommandManager( 
        Intake m_intake, 
        Index m_index, 
        Shooter m_shooter,
        LEDS m_leds,
        DriveSubsystem m_robotDrive,
        Climber m_climber,
        Vision2 m_vision) {

        m_lockAuto = new LockOnShooterAuto(m_shooter.m_pivot, m_vision, m_robotDrive, m_shooter);

        configureNamedCommands(
            m_intake, 
            m_index, 
            m_shooter,  
            m_leds,
            m_robotDrive,
            m_climber,
            m_vision);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0,
            new TrapezoidProfile.Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
                AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicDriveController = new HolonomicDriveController(
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController);

        TrajectoryConfig forwardConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(SwerveConstants.kDriveKinematics);

        TrajectoryConfig reverseConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(SwerveConstants.kDriveKinematics)
            .setReversed(true);

        // PathPlannerAuto FOA4PieceWing = new PathPlannerAuto("FOA 4 piece wing");
        // PathPlannerAuto FOAMidSpeaker2ring = new PathPlannerAuto("FOA Mid speaker 2 ring real");
        // PathPlannerAuto FOA4pssac = new PathPlannerAuto("FOA 4 piece source side all center");
        // PathPlannerAuto TurningTest = new PathPlannerAuto("Test Turning Auto");
        SequentialCommandGroup BlueWing2Piece = new BlueWing(m_robotDrive, this);

        m_chooser.setDefaultOption("None", new InstantCommand());

        // m_chooser.addOption("W Auto", FOA4PieceWing);
        // m_chooser.addOption("Mid Speaker 2 ring", FOAMidSpeaker2ring);
        // m_chooser.addOption("FOA4pssac", FOA4pssac);
        // m_chooser.addOption("Turning Test Auto", TurningTest);

        m_chooser.addOption("Blue Wing 2 Piece", BlueWing2Piece);

        SmartDashboard.putData("W Auto Chooser", m_chooser);
    }

    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }

    public Command getAutoManagerSelected(){
        return m_chooser.getSelected();
    }

    public SwerveControllerCommand trajectoryCommand(Trajectory trajectory, DriveSubsystem m_robotDrive) {
        return new SwerveControllerCommand(
            trajectory, 
            m_robotDrive::getEstimatedPose, 
            SwerveConstants.kDriveKinematics,
            holonomicDriveController,
            m_robotDrive::setModuleStates, 
            m_robotDrive);
    }

    public void configureNamedCommands(
        Intake m_intake,
        Index m_index, 
        Shooter m_shooter, 
        LEDS m_leds,
        DriveSubsystem m_robotDrive,
        Climber m_climber,
        Vision2 m_vision) { 

        NamedCommands.registerCommand("RunIntakeOld", new IntakeRun(m_intake, m_shooter, m_index, m_leds));
        NamedCommands.registerCommand("RunIntake", new IntakeSpinUp(m_intake, m_shooter, m_index, m_leds));
        NamedCommands.registerCommand("RunIntakeNew", new IntakeSpinUpAuto(m_intake, m_index, m_shooter, m_leds, m_robotDrive, m_lockAuto));
        m_runIntakeNew = new IntakeSpinUpAuto(m_intake, m_index, m_shooter, m_leds, m_robotDrive, m_lockAuto);
        NamedCommands.registerCommand("LockIn", new ParallelCommandGroup(Commands.runOnce(() -> m_robotDrive.isAutoRotate = true), Commands.runOnce(() -> m_lockAuto.enable())));
        NamedCommands.registerCommand("VisionShoot", (new VisionShoot(m_shooter, m_index, m_leds, m_vision, m_shooter.m_pivot, m_robotDrive)).withTimeout(2).finallyDo(() -> {m_robotDrive.isAutoRotate = false; m_lockAuto.reset();}));
        m_visionShoot = new VisionShoot(m_shooter, m_index, m_leds, m_vision, m_shooter.m_pivot, m_robotDrive).withTimeout(2).finallyDo(() -> {m_robotDrive.isAutoRotate = false; m_lockAuto.reset();});

        NamedCommands.registerCommand("ClimbDown", new ClimbDown(m_climber));
        NamedCommands.registerCommand("Start", new InstantCommand(() -> m_shooter.m_pivot.stopAngle()).andThen(new ClimbDown(m_climber).withTimeout(5)));
        NamedCommands.registerCommand("ShootRingWoofer", new InstantCommand (() -> m_shooter.setShooterStuff(56, 2500, "Woofer")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(2)));
        NamedCommands.registerCommand("ShootRingPodium", new InstantCommand (() -> m_shooter.setShooterStuff(24, 3240, "Podium")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));
        NamedCommands.registerCommand("ShootRingWing", new InstantCommand (() -> m_shooter.setShooterStuff(22, 3780, "Wing")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));
        NamedCommands.registerCommand("ShootRingMiddleStage", new InstantCommand (() -> m_shooter.setShooterStuff(22, 3510, "Middle Stage")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));
        NamedCommands.registerCommand("TurnOffWheels", Commands.runOnce(() -> m_robotDrive.setModuleStates(SwerveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)))));

        NamedCommands.registerCommand("4palc1Preset", new InstantCommand(() -> m_shooter.setShooterStuff(25.5, 3240, "4palc1"))); // First shot
        NamedCommands.registerCommand("4palc1", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(24, 3510, "4palc2")))); // Second shot
        NamedCommands.registerCommand("4palc2", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(22.5, 3780, "4palc3")))); // Third shot
        NamedCommands.registerCommand("4palc3", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3));
        
        // 4 piece source side all center
        NamedCommands.registerCommand("4pssac1Preset", new InstantCommand(() -> m_shooter.setShooterStuff(26, 3240, "4pssac1"))); // First shot
        NamedCommands.registerCommand("4pssac1", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(25.5, 3510, "4pssac2")))); // Second shot
        NamedCommands.registerCommand("4pssac2", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(24, 3780, "4pssac3")))); // Third shot
        NamedCommands.registerCommand("4pssac3", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3));
    }
}