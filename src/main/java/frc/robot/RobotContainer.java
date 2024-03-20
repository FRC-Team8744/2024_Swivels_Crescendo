// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.auto_led;
import frc.robot.commands.Trings;
import frc.robot.commands.TringsTest;
import frc.robot.commands.VisionShoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision2;
import frc.robot.commands.ShootA;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeSpinUp;
import frc.robot.commands.OuttakeRun;
import frc.robot.commands.SetLed;
import frc.robot.commands.ShootRing;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.TestPivot;
import frc.robot.commands.Wait;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.time.Instant;
import java.util.List;

import javax.security.auth.callback.TextInputCallback;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter();
  public final Vision m_vision = new Vision();
  public final Index m_index = new Index();
  public final Vision2 m_Vision2 = new Vision2();
  public final LEDS m_leds = new LEDS();
  public final Climber m_climber = new Climber();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // XboxController m_codriverController = new XboxController(OIConstants.kCodriverControllerPort);
  CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);

  // A chooser for autonomous commands
  private final SendableChooser<Command> m_autoChooser;
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(6);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // // Subsystem initialization
    // swerve = new Swerve();
    // exampleSubsystem = new ExampleSubsystem();

    m_leds.ledOn(0, 0, 255);

    // // Register Named Commands
    NamedCommands.registerCommand("RunIntakeOld", new IntakeRun(m_intake, m_shooter, m_index, m_leds));
    NamedCommands.registerCommand("RunIntake", new IntakeSpinUp(m_intake, m_shooter, m_index, m_leds));
    NamedCommands.registerCommand("Climb Down", new ClimbDown(m_climber));
    NamedCommands.registerCommand("Start", new InstantCommand(() -> m_shooter.stopAngle()).andThen(new ClimbDown(m_climber).withTimeout(5)));
    NamedCommands.registerCommand("ShootRingWoofer", new InstantCommand (() -> m_shooter.setShooterStuff(58, 2500, "Woofer")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(2)));
    NamedCommands.registerCommand("ShootRingPodium", new InstantCommand (() -> m_shooter.setShooterStuff(36, 3240, "Podium")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));
    NamedCommands.registerCommand("ShootRingWing", new InstantCommand (() -> m_shooter.setShooterStuff(22, 3780, "Wing")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));
    NamedCommands.registerCommand("ShootRingMiddleStage", new InstantCommand (() -> m_shooter.setShooterStuff(29, 3510, "Middle Stage")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));

    // 4 piece all amp center
    NamedCommands.registerCommand("4palc1Preset", new InstantCommand(() -> m_shooter.setShooterStuff(25.5, 3240, "4palc1"))); // First shot
    NamedCommands.registerCommand("4palc1", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(24, 3510, "4palc2")))); // Second shot
    NamedCommands.registerCommand("4palc2", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(22.5, 3780, "4palc3")))); // Third shot
    NamedCommands.registerCommand("4palc3", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3));
    
    // 4 piece source side all center
    NamedCommands.registerCommand("4pssac1Preset", new InstantCommand(() -> m_shooter.setShooterStuff(25.5, 3240, "4pssac1"))); // First shot
    NamedCommands.registerCommand("4pssac1", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(24.5, 3510, "4pssac2")))); // Second shot
    NamedCommands.registerCommand("4pssac2", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(23, 3780, "4pssac3")))); // Third shot
    NamedCommands.registerCommand("4pssac3", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3));

    // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_xspeedLimiter.calculate( -m_driverController.getLeftY() )*SwerveConstants.kMaxSpeedTeleop,
                    m_yspeedLimiter.calculate( -m_driverController.getLeftX() )*SwerveConstants.kMaxSpeedTeleop,
                    m_rotLimiter.calculate( -m_driverController.getRightX() )*ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                    true),
            m_robotDrive));

    // Add commands to the autonomous command chooser
    // m_chooser.setDefaultOption("SwerveCommand", SwerveCommand());
    // m_chooser.addOption("Swerve2Command", Swerve2Command());
    // m_chooser.addOption("PathPlannerCommand",PathPlannerCommand());
    m_autoChooser = AutoBuilder.buildAutoChooser();  // Default auto will be 'Commands.none()'
    // m_autoChooser.addOption("SwerveCommand", SwerveCommand());

    // Put the chooser on the dashboard
    // SmartDashboard.putData(m_chooser);
    SmartDashboard.putData("Auto Mode", m_autoChooser);

    // SmartDashboard.putData("SwerveBase", m_robotDrive);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kA.value)
    //   //.onTrue(new auto_led())
    //   .onFalse( m_led());
    // new JoystickButton(m_driverController, Button.kY.value)
    // .onTrue(new InstantCommand(() -> m_leds.ledOn()))
    // .onFalse(new InstantCommand(() -> m_leds.ledOff()));
    // .onFalse(new InstantCommand(() -> m_leds.ledOff()));
    // SmartDashboard.putData("SwerveCommand", new PathPlannerAuto("SwerveCommand"));
    m_driver.leftTrigger().whileTrue(new AmpShoot(m_climber, m_shooter, m_index, m_leds));
    m_driver.rightTrigger().whileTrue(new ShootRing(m_shooter, m_index, m_leds));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
    .whileTrue(new IntakeSpinUp(m_intake, m_shooter, m_index, m_leds));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
    .whileTrue(Commands.sequence(new auto_led(m_Vision2, m_robotDrive, m_leds, m_shooter).withTimeout(1.0), new VisionShoot(m_shooter, m_index, m_leds, m_Vision2)));

    new JoystickButton(m_driverController, Button.kX.value)
    .whileTrue(new OuttakeRun(m_intake, m_shooter, m_index));
    new JoystickButton(m_driverController, Button.kY.value)
    .whileTrue(new ClimbUp(m_climber));
    new JoystickButton(m_driverController, Button.kB.value)
    .whileTrue(new SourceIntake(m_shooter, m_index, m_leds));
    new JoystickButton(m_driverController, Button.kA.value)
    .whileTrue(new ClimbDown(m_climber));

    new POVButton(m_driverController, 0)
   .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(58, 2500, "Woofer")));
    new POVButton(m_driverController, 90)
    .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(30, 3240, "Podium")));
    new POVButton(m_driverController, 180)
    .whileTrue(new InstantCommand(() -> m_shooter.stopShooter()));
    new POVButton(m_driverController, 270)
    .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(45, 2630, "Shuffle")));
    
    new JoystickButton(m_driverController, Button.kBack.value)
    .whileTrue(new RunCommand(() -> m_robotDrive.zeroIMU()));
    new JoystickButton(m_driverController, Button.kLeftStick.value)
    .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.toggleMaxOutput()));

    // Codriver Bindings
    // new JoystickButton(m_codriverController, Button.kLeftBumper.value)
    // .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(60, 2500, "Woofer")));
    // new JoystickButton(m_codriverController, Button.kRightBumper.value)
    // .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(59, 1300, "Amp")));
    // new JoystickButton(m_codriverController, Button.kA.value)
    // .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(22, 3780, "Wing")));
    // new JoystickButton(m_codriverController, Button.kB.value)
    // .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(26, 3240, "Podium")));
    // new JoystickButton(m_codriverController, Button.kX.value)
    // .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(58, 2250, "Trap")));
    // new JoystickButton(m_codriverController, Button.kY.value)
    // .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(21, 4250, "Center")));
  //   new JoystickButton(m_driverController, Button.kB.values)
  //   .onTrue(new InstantCommand(() -> m_intake.donutGrab()))
  //   .onFalse(new InstantCommand(() -> m_intake.motorOff()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // public Command SwerveCommand() {
  //   m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

  //     // Create config for trajectory
  //   TrajectoryConfig config =
  //       new TrajectoryConfig(
  //               AutoConstants.kMaxSpeedMetersPerSecond,
  //               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //           // Add kinematics to ensure max speed is actually obeyed
  //           .setKinematics(SwerveConstants.kDriveKinematics);

  //   // An example trajectory to follow.  All units in meters.
  //   Trajectory exampleTrajectory =
  //       TrajectoryGenerator.generateTrajectory(
  //           // Start at the origin facing the +X direction
  //           new Pose2d(0, 0, new Rotation2d(0)),
  //           // Pass through these two interior waypoints, making an 's' curve path
  //           List.of(new Translation2d(1, 0),
  //                   new Translation2d(2, 0)),
  //           // End 3 meters straight ahead of where we started, facing forward
  //           new Pose2d(3, 0, new Rotation2d(0)),
  //           config);

  //   // m_robotDrive.m_field.getObject("traj").setTrajectory(exampleTrajectory);

  //   var thetaController =
  //       new ProfiledPIDController(
  //           AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
  //   SwerveControllerCommand swerveControllerCommand =
  //       new SwerveControllerCommand(
  //           exampleTrajectory,
  //           m_robotDrive::getPose, // Functional interface to feed supplier
  //           SwerveConstants.kDriveKinematics,

  //           // Position controllers
  //           new PIDController(AutoConstants.kPXController, 0, 0),
  //           new PIDController(AutoConstants.kPYController, 0, 0),
  //           thetaController,
  //           m_robotDrive::setModuleStates,
  //           m_robotDrive);

  //   // Reset odometry to the starting pose of the trajectory.
  //   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  //   // Run path following command, then stop at the end.
  //   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  // }

//   public Command Swerve2Command() {
//     // Create config for trajectory
//   TrajectoryConfig config =
//       new TrajectoryConfig(
//               AutoConstants.kMaxSpeedMetersPerSecond,
//               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//           // Add kinematics to ensure max speed is actually obeyed
//           .setKinematics(SwerveConstants.kDriveKinematics);

//   // An example trajectory to follow.  All units in meters.
//   Trajectory exampleTrajectory =
//       TrajectoryGenerator.generateTrajectory(
//           // Start at the origin facing the +X direction
//           new Pose2d(3, 0, new Rotation2d(0)),
//           // Pass through these two interior waypoints, making an 's' curve path
//           List.of(new Translation2d(2, -1),
//                   new Translation2d(1, 1)),
//           // End 3 meters straight ahead of where we started, facing forward
//           new Pose2d(0, 0, new Rotation2d(0)),
//           config);

//     // m_robotDrive.m_field.getObject("traj").setTrajectory(exampleTrajectory);

//   var thetaController =
//       new ProfiledPIDController(
//           AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//   thetaController.enableContinuousInput(-Math.PI, Math.PI);

//   SwerveControllerCommand swerveControllerCommand =
//       new SwerveControllerCommand(
//           exampleTrajectory,
//           m_robotDrive::getPose, // Functional interface to feed supplier
//           SwerveConstants.kDriveKinematics,

//           // Position controllers
//           new PIDController(AutoConstants.kPXController, 0, 0),
//           new PIDController(AutoConstants.kPYController, 0, 0),
//           thetaController,
//           m_robotDrive::setModuleStates,
//           m_robotDrive);

//   // Reset odometry to the starting pose of the trajectory.
//   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

//   // Run path following command, then stop at the end.
//   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
// }

//  public Command getAutonomousCommand() {return m_chooser.getSelected();}  //return SwerveCommand();}
 public Command getAutonomousCommand() {return m_autoChooser.getSelected();}  //return SwerveCommand();}

}