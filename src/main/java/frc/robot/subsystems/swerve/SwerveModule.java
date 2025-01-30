package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule extends SubsystemBase {
    private final double steerOffset = 0.0;
    // TODO: tune pid controllers
    private final PIDController drivePidController =
            new PIDController(1, 0, 0);

    private final ProfiledPIDController drivePpidController =
            new ProfiledPIDController(1, 0, 0, new Constraints(0.0, 0.0));

    private final PIDController steerPidController = new PIDController(1, 0, 0);

    private final ProfiledPIDController steerPpidController =
            new ProfiledPIDController(1, 0, 0, new Constraints(0.0, 0.0));

    private final SimpleMotorFeedforward steerFeedforwardController =
            new SimpleMotorFeedforward(1.0, 0.0, 0.0);

    /* Default to open loop. Closed loop is used for PID control over the driving rather than just the steering. */
    private boolean closedLoop = false;

    private double desiredVelocity = 0.0; // m/s
    private double currentVelocity = 0.0;

    private Rotation2d currentAngle = new Rotation2d();
    private Rotation2d desiredAngle = new Rotation2d();
    private double simDistance = 0.0;

    private final String moduleName;

    private final TalonFX driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder canCoder;

    // TODO: add relevant documentation or delete this if unnecessary
    public SwerveModule(String moduleName, int driveMotorID, int steerMotorID,
                        boolean driveInverted, boolean steerInverted,
                        int encoderID, double encoderOffset) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        this.canCoder = new CANcoder(encoderID);

        this.moduleName = moduleName;

        // Configure motors
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig
            .inverted(steerInverted)
            .idleMode(IdleMode.kBrake);

        // TODO finish steer motor configuration
        // TODO finish drive motor configuraiton

        // TODO: consider trying continuous input for steer controller because it handles shortest path
        // steerPidController.enableContinuousInput();
    }

    public void setState(SwerveModuleState state) {
        // SwerveModuleState desiredState = new SwerveModuleState(currentVelocity, currentAngle);
        // desiredState.optimize(currentAngle);
        state.optimize(currentAngle);
        desiredAngle = state.angle;
        desiredVelocity = state.speedMetersPerSecond;

        // SwerveModuleState desiredState = new SwerveModuleState(state.speedMetersPerSecond, currentAngle);
        // desiredAngle = desiredState.angle;

        // this.desiredVelocity = desiredState.speedMetersPerSecond;

        if (closedLoop) {
            drivePidController.setSetpoint(desiredVelocity);
            drivePpidController.setGoal(desiredVelocity);
        } else {
            // set percent scaled by max speed, drives full speed if desiredVelocity is m
            driveMotor.set(desiredVelocity / Constants.maxVelocity);
        }
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            (Robot.isSimulation()) ? simDistance : canCoder.getPosition().getValueAsDouble(),
            currentAngle
        );
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            currentVelocity,
            currentAngle
        );
    }

    /**
     *
     * @param closedLoop
     */
    public void setClosedLoop(boolean closedLoop) {
        this.closedLoop = closedLoop;
    }

    public double rotationsToRadians(double rotations) {
        return Rotation2d.fromRotations(rotations).getRadians();
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
            currentAngle = new Rotation2d(steerPidController.getSetpoint() + steerOffset);
        } else {
            currentAngle = new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble() + steerOffset);
        }

        SmartDashboard.putNumber("[" + moduleName + "] Current Angle", currentAngle.getDegrees());
        SmartDashboard.putNumber("[" + moduleName + "] Desired Angle", desiredAngle.getDegrees());

        steerPidController.setSetpoint(desiredAngle.getRadians());
        steerPpidController.setGoal(desiredAngle.getRadians());

        /* set the percentage of the steer motor controller to pid controller + feedforward
           controller scaled by voltage  */
        steerMotor.set(
                steerPidController.calculate(currentAngle.getRadians())
                /* computes static gain plus velocity gain multiplied by desired angle scaled by voltage
                   ks * signum(desired angle) + (kv * desired angle) / batter voltage
                   note, ka is set to 0 in this method overload so is omited */
                + steerFeedforwardController.calculate(
                        steerPidController.getSetpoint()
                ) / RobotController.getBatteryVoltage()
        );

        /** 
         * Update current velocity and 
         */
        if (Robot.isSimulation()) {
            currentVelocity = drivePidController.getSetpoint();
            simDistance += drivePidController.getSetpoint() * Robot.kDefaultPeriod;
        } else {
            if (canCoder == null) {
                currentVelocity = rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
            } else {
                currentVelocity = rotationsToRadians(canCoder.getVelocity().getValueAsDouble());
            }
        }
        if (closedLoop) {
            drivePidController.setSetpoint(currentVelocity);
        }

        SmartDashboard.putNumber("[" + moduleName + "] Current Velocity", currentVelocity);
        SmartDashboard.putNumber("[" + moduleName + "] Desired Velocity", desiredVelocity);
        SmartDashboard.putNumber("[" + moduleName + "] Velocity Error", Math.abs(desiredVelocity - currentVelocity));

        // TODO: Smartdashboard outputs
    }
}
