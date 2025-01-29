package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Supplier;

/** Arbitrary values used in this program are coppied from and this code base is highly referenced: <a href="https://github.dev/Mechanisms-Robotics/2024Robot/blob/main/src/main/java/com/mechlib/swerve/SwerveModule.java">2024Robot</a> **/

 /** Swerve subsystem interfaces with swerve hardware: motors, gyro, and CAN coders.
  * Declaration of constants related to the swerve. */
public class Swerve extends SubsystemBase {
    /* Pigeon 2 gyro. For the swerve so it can tell how fast it is moving.
       This is used to calculate PID values and for autonomous */ 
    private final Pigeon2 gyro = new Pigeon2(0);
    private final double width = 0.48;
    private final double length = 0.48;
    /* Swerve module locations, used by the program to compute kinematics going from a
       desired velocity and direction to the need motor outputs on each module. */
    private final Translation2d flModuleLocation = new Translation2d(width/2, length/2);
    private final Translation2d frModuleLocation = new Translation2d(width/2, -length/2);
    private final Translation2d blModuleLocation = new Translation2d(-width/2, -length/2);
    private final Translation2d brModuleLocation = new Translation2d(-width/2, length/2);
    /* Does math to get from desired velocity and direction to needed motor outputs */
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        flModuleLocation, frModuleLocation, blModuleLocation, brModuleLocation
    );

    private final Field2d field2d = new Field2d();

     // all these values are subject to change
     private final SwerveModule flModule = new SwerveModule("FL Module",
             1, 5,
             false, false,
             5, 0);

     private final SwerveModule frModule = new SwerveModule("FR Module",
             1, 6,
             false, false,
             5, 0);

     private final SwerveModule blModule = new SwerveModule("BL Module",
             1, 7,
             false, false,
             5, 0);

     private final SwerveModule brModule = new SwerveModule("BR Module",
             1, 8,
             false, false,
             5, 0);

    public Swerve() { 
        SmartDashboard.putData(field2d);
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

     /**
      * Drive at given x, y, and angular velocities
      *
      * @param vx x component velocity
      * @param vy y component velocity
      * @param omega angular momentum
      */
    public void drive(double vx, double vy, double omega) {
        // TODO add deadband
        // Computes math needed to get the motor outputs for each swerve module from the desired velocity and direction
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
        /* Desaturates wheel speeds, stops motors from going faster than they are able to which can happen in certain
           some swerve orientations that force some motors to dominate */
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxVelocity);
        setModuleStates(states);
    }

     /**
      * Drive the swerve in the desired direction and rotation.
      * Swerve drives at percentage of given velocities.
      * <p>* Teleop is currently in open loop</p>
      *
      * @param percentX x component supplier
      * @param percentY y component supplier
      * @param percentOmega angular momentum component supplier
      */
    public void teleopDrive(Supplier<Double> percentX, Supplier<Double> percentY, Supplier<Double> percentOmega) {
        drive(percentX.get() * Constants.maxVelocity,
              percentY.get() * Constants.maxVelocity,
              percentOmega.get() * Constants.maxAngularVelocity);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        flModule.setState(states[0]);
        frModule.setState(states[1]);
        blModule.setState(states[2]);
        brModule.setState(states[3]);
    }

    public void setOpenLoop() {
        flModule.setClosedLoop(false);
        frModule.setClosedLoop(false);
        blModule.setClosedLoop(false);
        brModule.setClosedLoop(false);
    }

    public void setClosedLoop() {
        flModule.setClosedLoop(true);
        frModule.setClosedLoop(true);
        blModule.setClosedLoop(true);
        brModule.setClosedLoop(true);
    }

    public void drawModules() {
        field2d.getObject("FL Module").setPose(
                new Pose2d()
        );
    }

    @Override
    public void periodic() {
        field2d.setRobotPose(new Pose2d());
        drawModules();
    }
}
