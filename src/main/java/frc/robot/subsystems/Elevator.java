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

    // Elevator positions in encoder tics
    // TODO: Determine experimentally and then put an equation for offline estimation of changes and put that here for posterity
    private final int RESTING = 0; // TODO make sure powered down completely when resting or very low to avoid power draw
    private final int PROCESSOR = 0;
    private final int LOADING = 0;
    private final int L1 = 0;
    private final int L2 = 0;
    private final int L3 = 0;
    private final int L4 = 0;
    private final int BARGE = 0;

    // TODO: Set CAN ids
    private final int LEADER_CAN_ID = 0;
    private final int FOLLOWER_CAN_ID = 0;

    // Motor controllers: one leader and one follower
    private final SparkMax m_leader;
    private final SparkMax m_follower;

    // The throughbore encoder is on the leader
    private final RelativeEncoder m_output_encoder;

    // Feedback (PID) and feedforward controllers
    private final PIDController m_pidController;
    private final SimpleMotorFeedforward m_feedforward;

    // Target elevator position
    private double m_targetPosition = 0.0;

    // PID gains TODO: Tune
    private static final double KP = 0.0;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double ERROR_TOLERANCE = 0.0;

    // Feedforward constants TODO: Tune
    // KS (static), KV (velocity), and KA (acceleration) terms.
    // For an elevator, you typically add a gravity term (KG) for compensation.
    private static final double KS = 0.0;
    private static final double KV = 0.0;
    private static final double KA = 0.0;
    // Gravity compensation term – adjust this to counteract the weight of the elevator
    private static final double KG = 0.0;

    /**
     * Constructs the Elevator subsystem.
     */
    public Elevator() {
        // Initialize motor controllers
        m_leader = new SparkMax(LEADER_CAN_ID, MotorType.kBrushless);
        m_follower = new SparkMax(FOLLOWER_CAN_ID, MotorType.kBrushless);

        // Configure the motors
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        followerConfig.follow(m_leader, true /* inverted */);

        m_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get the encoder from the leader (the throughbore encoder on the output shaft)
        m_output_encoder = m_leader.getAlternateEncoder();

        // Initialize the PID controller for feedback control
        m_pidController = new PIDController(KP, KI, KD);
        m_pidController.setTolerance(ERROR_TOLERANCE); // Adjust tolerance as needed

        // Initialize the feedforward controller.
        // For a stationary elevator hold, we assume target velocity = 0.
        m_feedforward = new SimpleMotorFeedforward(KS, KV, KA);
    }

    /**
     * Sets the desired elevator position.
     *
     * @param position The target position
     */
    public void setTargetPosition(double position) {
        m_targetPosition = position;
    }

    /**
     * Returns the current elevator position.
     *
     * @return The current position
     */
    public double getCurrentPosition() {
        return m_output_encoder.getPosition();
    }

    @Override
    public void periodic() {
        // Get the current position from the encoder
        double currentPosition = getCurrentPosition();

        // Calculate the feedback output from the PID controller.
        double feedbackOutput = m_pidController.calculate(currentPosition, m_targetPosition);

        // Calculate feedforward. For a stationary target (holding position),
        // we assume zero target velocity.
        double ffOutput = m_feedforward.calculate(0.0) + KG;

        // Combine feedback and feedforward.
        double motorOutput = feedbackOutput + ffOutput;

        // Clamp the motor output to the valid range [-1, 1]
        motorOutput = Math.max(-1.0, Math.min(1.0, motorOutput));

        // Command the motor
        m_leader.set(motorOutput);

        // (Optional) Output diagnostic info
        System.out.println("Elevator -> Target: " + m_targetPosition 
            + ", Position: " + currentPosition 
            + ", Motor Output: " + motorOutput);
    }
}
