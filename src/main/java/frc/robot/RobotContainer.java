// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.auto_led;
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
import frc.robot.commands.SourceIntake;
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
  public DriveSubsystem m_robotDrive;
  public Intake m_intake;
  public Shooter m_shooter;
  public Index m_index;
  public Vision2 m_Vision2;
  public LEDS m_leds;
  public Climber m_climber;

  // The driver's controller
  XboxController m_driverController;
  XboxController m_codriverController;
  CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);

  // A chooser for autonomous commands
  private SendableChooser<Command> m_autoChooser;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(6);

  private final Who iAm;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystem initialization
    iAm = Who.iAm();

    switch (iAm) {
      case NO_NO:
        m_leds = new LEDS();
        m_intake = new Intake();
        m_shooter = new Shooter();
        m_index = new Index();
        m_Vision2 = new Vision2();
        m_climber = new Climber();
      case SWIVELS:
        m_robotDrive = new DriveSubsystem();
        break;
      case UNDEFINED:
        throw new Error("The Robot Name is not Defined");
      default:
        break;
    }

    switch(iAm) {
      case NO_NO:
        m_leds.ledOn(0, 0, 255);
        
        // Register Named Commands
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
        break;
      default:
        break;
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    initializeDriver();

    SmartDashboard.putData("Start Codriver", Commands.runOnce(this::initializeCodriver));
  }

  // Driver Bindings
  private void initializeDriver() {
    m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    switch(iAm) {
      case NO_NO:
        m_driver.leftTrigger().whileTrue(new AmpShoot(m_climber, m_shooter, m_index, m_leds));
        m_driver.rightTrigger().whileTrue(new ShootRing(m_shooter, m_index, m_leds));

        new JoystickButton(m_driverController, Button.kLeftBumper.value)
          .whileTrue(new IntakeSpinUp(m_intake, m_shooter, m_index, m_leds));
        new JoystickButton(m_driverController, Button.kRightBumper.value)
          .whileTrue(Commands.sequence(new auto_led(m_Vision2, m_robotDrive, m_leds, m_shooter).withTimeout(1.0), new VisionShoot(m_shooter, m_index, m_leds, m_Vision2)));

        new JoystickButton(m_driverController, Button.kX.value).whileTrue(new OuttakeRun(m_intake, m_shooter, m_index));
        new JoystickButton(m_driverController, Button.kY.value).whileTrue(new ClimbUp(m_climber));
        new JoystickButton(m_driverController, Button.kB.value).whileTrue(new SourceIntake(m_shooter, m_index, m_leds));
        new JoystickButton(m_driverController, Button.kA.value).whileTrue(new ClimbDown(m_climber));

    new POVButton(m_driverController, 0)
   .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(56, 2500, "Woofer")));
    new POVButton(m_driverController, 90)
    .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(30, 3240, "Podium")));
    new POVButton(m_driverController, 180)
    .whileTrue(new InstantCommand(() -> m_shooter.stopShooter()));
    new POVButton(m_driverController, 270)
    .onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(45, 2630, "Shuffle")));
    
        new JoystickButton(m_driverController, Button.kBack.value).whileTrue(new RunCommand(() -> m_robotDrive.zeroIMU()));
        new JoystickButton(m_driverController, Button.kLeftStick.value).toggleOnTrue(Commands.runOnce(() -> m_robotDrive.toggleMaxOutput()));
        break;
      case SWIVELS:
        break;
      default:
        break;
    }
  }

  // Codriver Bindings
  private void initializeCodriver() {
    m_codriverController = new XboxController(OIConstants.kCodriverControllerPort);
    switch(iAm) {
      case NO_NO:
        new JoystickButton(m_codriverController, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(60, 2500, "Woofer")));
        new JoystickButton(m_codriverController, Button.kRightBumper.value).onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(59, 1300, "Amp")));
        new JoystickButton(m_codriverController, Button.kA.value).onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(22, 3780, "Wing")));
        new JoystickButton(m_codriverController, Button.kB.value).onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(26, 3240, "Podium")));
        new JoystickButton(m_codriverController, Button.kX.value).onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(58, 2250, "Trap")));
        new JoystickButton(m_codriverController, Button.kY.value).onTrue(new InstantCommand(() -> m_shooter.setShooterStuff(21, 4250, "Center")));
        new JoystickButton(m_driverController, Button.kB.value).whileTrue(new IntakeRun(m_intake, m_shooter, m_index, m_leds));
        break;
      case SWIVELS:
        break;
      default:
        break;
    }
  }
 
  public Command getAutonomousCommand() {
    switch(iAm) {
      case NO_NO:
        return m_autoChooser.getSelected();
      case SWIVELS:
      default:
        return Commands.none();
    }
  }
}