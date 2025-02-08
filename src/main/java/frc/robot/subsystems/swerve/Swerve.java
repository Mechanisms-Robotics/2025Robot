package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

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

    private final PIDController stabilizeController;

    private final Field2d field = new Field2d();

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

     private Rotation2d simHeading = new Rotation2d();

     private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d()
     );

    public Swerve() {
        SmartDashboard.putData(field);

        stabilizeController = new PIDController(0, 0, 0);
        stabilizeController.setTolerance(0.1); // ???

        System.out.println(Constants.config);
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getSpeeds, 
            this::autoDrive,
            new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0) // default, no particular reason for this
                ),
            Constants.config,    
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this
            );
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
        omega = 0.1;

        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("omega", omega);

        // Computes math needed to get the motor outputs for each swerve module from the desired velocity and direction
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
        /* Desaturates wheel speeds, stops motors from going faster than they are able to which can happen in certain
           some swerve orientations that force some motors to dominate */
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxVelocity);
        setModuleStates(states);

        //if (Robot.isSimulation()) {
            //simHeading = simHeading.rotateBy(new Rotation2d(omega * Robot.kDefaultPeriod));
        //}
    }

    public void autoDrive(ChassisSpeeds speeds) {
        // if (Robot.isSimulation()) {
        //     speeds.vxMetersPerSecond /= 2.0;
        //     speeds.vyMetersPerSecond /= 2.0;
        // }

        // SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxVelocity);

        // setModuleStates(desiredStates);

        // if (Robot.isSimulation()) {
        //     simHeading = simHeading.rotateBy(new Rotation2d(
        //         speeds.omegaRadiansPerSecond * Robot.kDefaultPeriod
        //     ));
        // }
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
        SmartDashboard.putNumber("fl angle", states[0].angle.getDegrees());
        SmartDashboard.putNumber("fl speed", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("fr angle", states[1].angle.getDegrees());
        SmartDashboard.putNumber("fr speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("bl angle", states[2].angle.getDegrees());
        SmartDashboard.putNumber("bl speed", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("br angle", states[3].angle.getDegrees());
        SmartDashboard.putNumber("br speed", states[3].speedMetersPerSecond);

        flModule.setState(states[0]);
        frModule.setState(states[1]);
        blModule.setState(states[2]);
        brModule.setState(states[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                flModule.getModulePosition(),
                frModule.getModulePosition(),
                brModule.getModulePosition(),
                blModule.getModulePosition()
        };
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
        // if (Robot.isSimulation()) {
        //     simHeading = pose.getRotation();
        // }
    }

    public Rotation2d getHeading() {
        // if (Robot.isSimulation())
        //     return simHeading;
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()).rotateBy(new Rotation2d());
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

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            flModule.getModuleState(),
            frModule.getModuleState(),
            blModule.getModuleState(),
            brModule.getModuleState()
        };
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void drawModules() {
        field.getObject("FL Module").setPose(
            new Pose2d(
                getPose().getTranslation().plus(flModuleLocation.rotateBy(getHeading())),
                flModule.getModulePosition().angle.rotateBy(getHeading())
            )
        );

        field.getObject("FR Module").setPose(
            new Pose2d(
                getPose().getTranslation().plus(frModuleLocation.rotateBy(getHeading())),
                flModule.getModulePosition().angle.rotateBy(getHeading())
            )
        );

        field.getObject("BL Module").setPose(
            new Pose2d(
                getPose().getTranslation().plus(blModuleLocation.rotateBy(getHeading())),
                flModule.getModulePosition().angle.rotateBy(getHeading())
            )
        );

        field.getObject("BR Module").setPose(
            new Pose2d(
                getPose().getTranslation().plus(brModuleLocation.rotateBy(getHeading())),
                flModule.getModulePosition().angle.rotateBy(getHeading())
            )
        );
    }

    @Override
    public void periodic() {
        poseEstimator.update(getHeading(), getModulePositions());

        SmartDashboard.putNumber("[Swerve] x", getPose().getX());
        SmartDashboard.putNumber("[Swerve] y", getPose().getY());
        SmartDashboard.putNumber("[Swerve] Heading", getHeading().getDegrees());

        field.setRobotPose(getPose());
        drawModules();
    }
}
