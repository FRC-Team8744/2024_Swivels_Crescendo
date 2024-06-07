// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.Vision2;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeSpinUp;
import frc.robot.commands.OuttakeRun;
import frc.robot.commands.ShootRing;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

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
  public final Index m_index = new Index();
  public final Vision2 m_Vision2 = new Vision2();
  public final LEDS m_leds = new LEDS();
  public final Climber m_climber = new Climber();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);
  // XboxController m_codriverController = new XboxController(OIConstants.kCodriverControllerPort);
  CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);

  // A chooser for autonomous commands
  private final SendableChooser<Command> m_autoChooser;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

  private final String controllerMode = "x";

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_leds.ledOn(0, 0, 255);

    // // Register Named Commands
    NamedCommands.registerCommand("RunIntakeOld", new IntakeRun(m_intake, m_shooter, m_index, m_leds));
    NamedCommands.registerCommand("RunIntake", new IntakeSpinUp(m_intake, m_shooter, m_index, m_leds));
    NamedCommands.registerCommand("Climb Down", new ClimbDown(m_climber));
    NamedCommands.registerCommand("Start", new InstantCommand(() -> m_shooter.stopAngle()).andThen(new ClimbDown(m_climber).withTimeout(5)));
    NamedCommands.registerCommand("ShootRingWoofer", new InstantCommand (() -> m_shooter.setShooterStuff(56, 2500, "Woofer")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(2)));
    NamedCommands.registerCommand("ShootRingPodium", new InstantCommand (() -> m_shooter.setShooterStuff(36, 3240, "Podium")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));
    NamedCommands.registerCommand("ShootRingWing", new InstantCommand (() -> m_shooter.setShooterStuff(22, 3780, "Wing")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));
    NamedCommands.registerCommand("ShootRingMiddleStage", new InstantCommand (() -> m_shooter.setShooterStuff(29, 3510, "Middle Stage")).andThen(new ShootRing(m_shooter, m_index, m_leds).withTimeout(3)));

    // 4 piece all amp center
    NamedCommands.registerCommand("4palc1Preset", new InstantCommand(() -> m_shooter.setShooterStuff(25.5, 3240, "4palc1"))); // First shot
    NamedCommands.registerCommand("4palc1", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(24, 3510, "4palc2")))); // Second shot
    NamedCommands.registerCommand("4palc2", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(22.5, 3780, "4palc3")))); // Third shot
    NamedCommands.registerCommand("4palc3", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3));
    
    // 4 piece source side all center
    NamedCommands.registerCommand("4pssac1Preset", new InstantCommand(() -> m_shooter.setShooterStuff(26, 3240, "4pssac1"))); // First shot
    NamedCommands.registerCommand("4pssac1", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(25.5, 3510, "4pssac2")))); // Second shot
    NamedCommands.registerCommand("4pssac2", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3).andThen(new InstantCommand (() -> m_shooter.setShooterStuff(24, 3780, "4pssac3")))); // Third shot
    NamedCommands.registerCommand("4pssac3", new ShootRing(m_shooter, m_index, m_leds).withTimeout(3));

    // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
  if (controllerMode == "x") {
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
  }
  else if(controllerMode == "j") {
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () ->
          m_robotDrive.drive(
                m_xspeedLimiter.calculate( -m_Joystick.getRawAxis(1) )*SwerveConstants.kMaxSpeedTeleop,
                m_yspeedLimiter.calculate( -m_Joystick.getRawAxis(0) )*SwerveConstants.kMaxSpeedTeleop,
                m_rotLimiter.calculate( -m_Joystick.getRawAxis(2) )*ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                true),
        m_robotDrive));
  }
    m_autoChooser = AutoBuilder.buildAutoChooser();  // Default auto will be 'Commands.none()'

    SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    if (controllerMode == "x") {
      new JoystickButton(m_driverController, Button.kB.value)
      .onTrue(new auto_led(m_Vision2, m_robotDrive, m_leds, m_shooter).withTimeout(2.0));


      new JoystickButton(m_driverController, Button.kA.value)
      .onTrue(new Trings(m_leds, m_robotDrive));

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
      .whileTrue(new auto_led(m_Vision2, m_robotDrive, m_leds, m_shooter));
      new JoystickButton(m_driverController, Button.kA.value)
      .whileTrue(new ClimbDown(m_climber));
      new POVButton(m_driverController, 180)
      .whileTrue(new InstantCommand(() -> m_shooter.stopShooter()));
      new POVButton(m_driverController, 0)
      .whileTrue(new TringsTest(m_leds, m_robotDrive, m_Vision2));
  

      new POVButton(m_driverController ,0)
      .whileTrue(new IntakeRun(m_intake, m_shooter, m_index, m_leds));
      new JoystickButton(m_driverController, Button.kBack.value)
      .whileTrue(new RunCommand(() -> m_robotDrive.zeroIMU()));
      new JoystickButton(m_driverController, Button.kLeftStick.value)
      .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.toggleMaxOutput()));
    }
    else if(controllerMode == "j") {
      new JoystickButton(m_Joystick, 1)
      .whileTrue(new ShootRing(m_shooter, m_index, m_leds));
      new JoystickButton(m_Joystick, 2)
      .whileTrue(new IntakeSpinUp(m_intake, m_shooter, m_index, m_leds));
      new JoystickButton(m_Joystick, 6)
      .whileTrue(new ClimbUp(m_climber));
      new JoystickButton(m_Joystick, 4)
      .whileTrue(new ClimbDown(m_climber));
    }
  }

 public Command getAutonomousCommand() {return m_autoChooser.getSelected();}
}