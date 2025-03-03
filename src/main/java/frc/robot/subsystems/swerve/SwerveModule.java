package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
            new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

    private final PIDController steerPidController = new PIDController(0.25, 0, 0);

    private final ProfiledPIDController steerPpidController =
            new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

    private final SimpleMotorFeedforward steerFeedforwardController =
            new SimpleMotorFeedforward(0, 0, 0.0);

    /* Default to open loop. Closed loop is used for PID control over the driving rather than just the steering. */
    private boolean closedLoop = false;
    private static final double gearRatio = 6.12;

    private double desiredVelocity = 0.0; // m/s
    private double currentVelocity = 0.0;

    private Rotation2d currentAngle = new Rotation2d();
    private Rotation2d desiredAngle = new Rotation2d();
    private double simDistance = 0.0;

    private final String moduleName;

    private final TalonFX driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder canCoder;
    private double x = 0;

    // TODO: steerFeedforwardController
    
    // TODO: add relevant documentation or delete this if unnecessary
    public SwerveModule(String moduleName, int driveMotorID, int steerMotorID,
    boolean driveInverted, boolean steerInverted,
    int encoderID, double encoderOffset) {
        
        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        this.canCoder = new CANcoder(encoderID);
        
        this.moduleName = moduleName;
        SmartDashboard.putData(this.moduleName + "/Drive PID Controller", drivePidController);
        SmartDashboard.putData(this.moduleName + "/Drive Profiled PID Controller", drivePpidController);
        SmartDashboard.putData(this.moduleName + "/Steer PID Controller", drivePidController);
        SmartDashboard.putData(this.moduleName + "/Steer Profiled PID Controller", steerPpidController);    
        SmartDashboard.putData(this.moduleName + "/Steer Feedforward", steerPpidController);
        
        // Configure motors, unsuccessfull
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig
            .inverted(steerInverted)
            .idleMode(IdleMode.kBrake);

        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        if (driveInverted) {
            driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        
        driveMotor.getConfigurator().apply(driveConfig);

        // attempted cancoder config didn't work
        // MagnetSensorConfigs canCoderConfig = new MagnetSensorConfigs();
        // if (steerInverted) {
        //     canCoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // } else {
        //     canCoderConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // }
        // canCoder.getConfigurator().apply(canCoderConfig);

        // steerPidController.enableContinuousInput(0, 180);
    }

    public void setState(SwerveModuleState state) {
        state.optimize(currentAngle);
        desiredAngle = state.angle;
        desiredVelocity = state.speedMetersPerSecond;


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

    public void steerTo(Rotation2d angle) {
        desiredAngle = angle;
    }

    /**
     *
     * @param closedLoop
     */
    public void setClosedLoop(boolean closedLoop) {
        this.closedLoop = closedLoop;
    }

    // TODO: wpi units
    public double rotationsToRadians(double rotations) {
        return Rotation2d.fromRotations(rotations).getRadians();
    }

    public void testHalfPower() {
        driveMotor.set(.5);
    }
    public void test0Voltage() {
        driveMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(moduleName + "/CANcoder/absolute position", canCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber(moduleName + "/CANcoder/position since boot", canCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(moduleName + "/CANcoder/id", canCoder.getDeviceID());
        SmartDashboard.putNumber(moduleName + "/CANcoder/velocity", canCoder.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(moduleName + "/CANcoder/supply voltage", canCoder.getSupplyVoltage().getValueAsDouble());
        if (Robot.isSimulation()) {
            currentAngle = new Rotation2d(steerPidController.getSetpoint() + steerOffset);
        } else {
            currentAngle = Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
        }

        SmartDashboard.putNumber(moduleName + "/Current Angle", currentAngle.getDegrees());
        SmartDashboard.putNumber(moduleName + "/Desired Angle", desiredAngle.getDegrees());

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

        SmartDashboard.putNumber(moduleName + "/Current Velocity", currentVelocity);
        SmartDashboard.putNumber(moduleName + "/Desired Velocity", desiredVelocity);
        SmartDashboard.putNumber(moduleName + "/Velocity Error", Math.abs(desiredVelocity - currentVelocity));

        // TODO: Smartdashboard outputs
    }
}
