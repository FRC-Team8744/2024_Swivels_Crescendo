package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

import java.util.Arrays;

/* Use this enum to toggle robot features based on name */
public enum Who {
    NO_NO("nono"),
    SWIVELS("swivels"),
    UNDEFINED("undefined");

    private static final String PREFERENCE_NAME = "RobotName";

    private static Who I_AM = iAm(Preferences.getString(PREFERENCE_NAME, UNDEFINED.robotName));

    private final String robotName;

    private Who(String robotName) {
        this.robotName = robotName;
    }

    /**
     * Returns the name of the robot
     * @return
     */
    public String getRobotName() {
        return robotName;
    }

    /**
     * Returns what the robot name is currently set to
     * @return
     */
    public static Who iAm() {
        return I_AM;
    }

    /**
     * Used to ask if the robot is currently set to a name
     * @param areYou - Then name being asked if robot is named 
     * @return - If the robot is currently set to the provided name
     */
    public static boolean amI(Who areYou) {
        return I_AM == areYou;
    }

    /**
     * Sets the robot to the passed in name and returns the resulting robot name
     * @param iAm - The name to set the robot to
     * @return
     */
    static Who iAm(Who iAm) {
        Preferences.setString(PREFERENCE_NAME, iAm.robotName);
        I_AM = iAm;
        return I_AM;
    }

    /**
     * Sets the robot to the passed in name and returns the resulting robot name
     * @param robotName - The name to set the robot to
     * @return - If the robotName doesn't match one of the know names, UNDEFINED will be returned
     */
    static Who iAm(String robotName) {
        Preferences.setString(PREFERENCE_NAME, robotName);
        I_AM = Arrays.stream(Who.values())
            .filter((who) -> who.robotName.equals(robotName))
            .findAny()
            .orElse(UNDEFINED);
        return I_AM;
    }
}
