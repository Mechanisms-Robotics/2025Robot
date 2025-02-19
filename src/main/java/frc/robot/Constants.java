package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    public static final boolean usingKeyboard = true; // only applies to sim

    public static enum Mode {
        REAL,
        SIM,
    }
    /* Max velocity we want the robot to ever attempt, used in swerve math to stop it from calculating anything
       higher. This can happen in certain swerve orientations in which some wheels are used more than others. */
    public static final double maxVelocity = 4.25; // m/s
    public static final double maxAngularVelocity = 3 * Math.PI;
    public static final double weightPounds = 96;
    public static RobotConfig config;
    static {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        };
    };
}

