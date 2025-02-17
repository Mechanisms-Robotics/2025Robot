package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    // TODO: Set soft limits
    // TODO: Limit switches (see getForwardLimitSwitch)

    // TODO: Set CAN ids
    private final int leaderID = 0;
    private final int followerID = 0;

    // Motor controllers: one leader and one follower
    private final SparkMax leader;
    private final SparkMax follower;

    // The throughbore encoder is on the leader
    private final RelativeEncoder encoder;

    // Feedback (PID) and feedforward controllers
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforward;

    // Target elevator positionv
    private double targetPosition = 0.0;

    // --- Tuning Constants ---
    // PID gains (tune these to your mechanism)
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Feedforward constants
    // kS (static), kV (velocity), and kA (acceleration) terms.
    // For an elevator, you typically add a gravity term (kG) for compensation.
    private static final double kS = 0.0;
    private static final double kV = 0.0;
    private static final double kA = 0.0;
    // Gravity compensation term – adjust this to counteract the weight of the elevator
    private static final double kG = 0.5;

    // Conversion factor for the encoder (e.g., rotations to inches)
    //private static final double POSITION_CONVERSION_FACTOR = 1.0;

    /**
     * Constructs the Elevator subsystem.
     * 
     * @param leaderID   CAN ID for the leader motor controller.
     * @param followerID CAN ID for the follower motor controller.
     */
    public Elevator() {
        // Initialize motor controllers (assumed to be brushless for NEO motors)
        leader = new SparkMax(leaderID, MotorType.kBrushless);
        follower = new SparkMax(followerID, MotorType.kBrushless);

        // Configure the motors
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        followerConfig.follow(leader, true /* inverted */);

        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get the encoder from the leader (the throughbore encoder on the output shaft)
        encoder = leader.getAlternateEncoder();
        //encoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);

        // Initialize the PID controller for feedback control
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(0.1); // Adjust tolerance as needed

        // Initialize the feedforward controller.
        // For a stationary elevator hold, we assume target velocity = 0.
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    /**
     * Sets the desired elevator position.
     *
     * @param position The target position (in the same units as the conversion factor).
     */
    public void setTargetPosition(double position) {
        targetPosition = position;
    }

    /**
     * Returns the current elevator position.
     *
     * @return The current position (in the same units as conversion factor).
     */
    public double getCurrentPosition() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        // Get the current position from the encoder
        double currentPosition = getCurrentPosition();

        // Calculate the feedback output from the PID controller.
        double feedbackOutput = pidController.calculate(currentPosition, targetPosition);

        // Calculate feedforward. For a stationary target (holding position),
        // we assume zero target velocity.
        double ffOutput = feedforward.calculate(0.0) + kG;

        // Combine feedback and feedforward.
        double motorOutput = feedbackOutput + ffOutput;

        // Clamp the motor output to the valid range [-1, 1]
        motorOutput = Math.max(-1.0, Math.min(1.0, motorOutput));

        // Command the motor
        leader.set(motorOutput);

        // (Optional) Output diagnostic info
        System.out.println("Elevator -> Target: " + targetPosition 
            + ", Position: " + currentPosition 
            + ", Motor Output: " + motorOutput);
    }
}
