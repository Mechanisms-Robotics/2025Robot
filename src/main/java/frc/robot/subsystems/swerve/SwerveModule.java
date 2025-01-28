package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule extends SubsystemBase {
    private final double steerOffset = 0.0;
    // TODO: tune pid controllers
    private final PIDController drivePidController =
            new PIDController(0, 0, 0);

    private final ProfiledPIDController drivePpidController =
            new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

    private final PIDController steerPidController = new PIDController(0, 0, 0);

    private final ProfiledPIDController steerPpidController =
            new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

    private final SimpleMotorFeedforward steerFeedforwardController =
            new SimpleMotorFeedforward(0.0, 0.0, 0.0);

    /* Default to open loop. Closed loop is used for PID control over the driving rather than just the steering. */
    private boolean closedLoop = false;

    private double desiredVelocity = 0.0; // m/s

    private Rotation2d currentAngle = new Rotation2d();
    private Rotation2d desiredAngle = new Rotation2d();

    private final TalonFX driveMotor;
    private final SparkBase steerMotor;
    private final CANcoder canCoder;

    // TODO: add relevant documentation or delete this if unnecessary
    public SwerveModule(String moduleName, int driveMotorID, int steerMotorID,
                        boolean driveInverted, boolean steerInverted,
                        int encoderID, double encoderOffset) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        this.canCoder = new CANcoder(encoderID);

        // Configure motors

        SparkBaseConfig steerConfig = new SparkMaxConfig();
        steerConfig
            .inverted(steerInverted)
            .idleMode(IdleMode.kBrake);
    
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // steerMotor.setIdleMode(IdleMode.kBrake);

        // TODO finish steer motor configuration
        // TODO finish drive motor configuraiton
        // TODO: consider trying continuous input for steer controller because it handles shortest path
        // steerPidController.enableContinuousInput();
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, currentAngle);
        steerTo(desiredState.angle); // same as desiredAngle = desiredState.angle
        this.desiredVelocity = desiredState.speedMetersPerSecond;

        if (closedLoop) {
            drivePidController.setSetpoint(desiredVelocity);
            drivePpidController.setGoal(desiredVelocity);
        } else {
            // set percent scaled by max speed, drives full speed if desiredVelocity is m
            driveMotor.set(desiredVelocity / Constants.maxVelocity);
        }
    }

    /**
     * Desired angle modifier
     *
     * @param desiredAngle new desired angle
     */
    public void steerTo(Rotation2d desiredAngle) {
        this.desiredAngle = desiredAngle;
    }

    /**
     *
     * @param closedLoop
     */
    public void setClosedLoop(boolean closedLoop) {
        this.closedLoop = closedLoop;
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
            currentAngle = new Rotation2d(steerPidController.getSetpoint() + steerOffset);
        } else {
            currentAngle = new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble() + steerOffset);
        }

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

        if (closedLoop) {
            /** TODO: Set the output of the drive motor
                pidfOutput = 
                    PID controller(drive motor pid controller setpoint)
                    + feed forward controller(pid controller setpoint) / battery voltage
                driveMotor.set(pidfOutput) this will set the percent output of the motor
                reference: <a href=https://github.dev/Mechanisms-Robotics/2024Robot/blob/main/src/main/java/com/mechlib/swerve/SwerveModule.java>2024Robot</a>
                           <a href=https://github.dev/FRC-4509-MechBulls/2024_Crescendo_Bot> Mechbulls 2024 Robot </a>
            */
        }

        // TODO: Smartdashboard outputs
    }
}
