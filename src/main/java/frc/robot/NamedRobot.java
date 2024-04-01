package frc.robot;

import java.util.Arrays;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Implement this method interface to add the option to handle multiple robots
 */
public class NamedRobot {
    /**
     * This is the name of the chooser that will hold the list of robots
     */
    private static final String ROBOT_CHOOSE_NAME = "Choose Robot";

    private static Runnable createRobotContainer;

    private NamedRobot() {
        // This is hear because this is a static class and an instance shouldn't be created
    }

    /**
     * Add a call to this method where the robot is initialized.
     * Once a robot is selected, the Choose Robot option will be empty. use {@link NamedRobot#rename(Runnable)}
     * to rename after that.
     * @param createRobotContaier - this is used after a robot is selected to initialize the Robot Container
     */
    public static void init(Runnable createRobotContainer) {
        NamedRobot.init(createRobotContainer, false);
    }

    /**
     * NOTE: THIS METHOD SHOULD ONLY BE USED FOR RENAME A ROBOT!
     * Add a call to this method where the robot is initialized. 
     * Use {@link NamedRobot#init(Runnable)} as the typeical initializer.
     * @param createRobotContaier - this is used after a robot is selected to initialize the Robot Container
     */
    public static void rename(Runnable createRobotContainer) {
        NamedRobot.init(createRobotContainer, true);
    }

    /**
     * Add a call to this method where the robot is initialized.
      
     * @param createRobotContaier - this is used after a robot is selected to initialize the Robot Container
     * @param rename - used to override an already selected name for the robot.
     */
    public static void init(Runnable createRobotContainer, boolean rename) {
        NamedRobot.createRobotContainer = createRobotContainer;
        // NamedRobot.renamed = rename;

        if (Who.iAm() == Who.UNDEFINED || rename) {
            NamedRobot.initRobotChooser();
          } else {
            NamedRobot.initRobotContainer();
          }
    }

    /**
     * This method is used to create the robot name chooser on the smart dashboard
     */
    private static void initRobotChooser() {
        SendableChooser<Command> robotChooser = new SendableChooser<>();

        Arrays.stream(Who.values()).forEach(NamedRobot.addRobotChooserOption(robotChooser));

        robotChooser.onChange((who) -> {
            if (!LiveWindow.isEnabled()) {
                who.initialize();
                who.execute();
            }
        });

        SmartDashboard.putData(NamedRobot.ROBOT_CHOOSE_NAME, robotChooser);
        SmartDashboard.clearPersistent(NamedRobot.ROBOT_CHOOSE_NAME);
    }

    /**
     * This method is used to add robot names to the list for choosing on the smart board.
     * @param robotChooser - This is the robot chooser
     * @return - A Consumer function for the name.
     */
    private static Consumer<Who> addRobotChooserOption(SendableChooser<Command> robotChooser) {
        return who -> {
            Command command = Commands.runOnce(initRobotContainer(who)).andThen(robotChooser::close);
            if (who == Who.iAm()) {
                robotChooser.setDefaultOption(who.getRobotName(), command);
            } else {
                robotChooser.addOption(who.getRobotName(), command);
            }
        };
    }

    /**
     * This method create a function to apply the name to the robot and start up the rest of the robot
     * @param who - The robot name to be initialized
     * @return - A function for naming and initalizing the robot
     */
    private static Runnable initRobotContainer(Who who) {
        return () -> {
            Who.iAm(who);
            NamedRobot.initRobotContainer();
        };
    }

    /**
     * This method is used to initalized the robot
     */
    private static synchronized void initRobotContainer() {
        if (Who.iAm() != Who.UNDEFINED) {
            NamedRobot.createRobotContainer.run();
            SmartDashboard.updateValues();
        }
    }
}
