package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

public class Constants {

    /* Max velocity we want the robot to ever attempt, used in swerve math to stop it from calculating anything
       higher. This can happen in certain swerve orientations in which some wheels are used more than others. */
    public static final double maxVelocity = 4.25; // m/s
    public static final double maxAngularVelocity = 6 * 2; // tau is 2 pi, or one rotation
    public static final double weightPounds = 80;
    public static final double weightKilos = poundsToKg(weightPounds);
    public static double poundsToKg(double pounds) { return pounds * 0.4536; };    
    public static RobotConfig config;
    static {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        };
    };
}

