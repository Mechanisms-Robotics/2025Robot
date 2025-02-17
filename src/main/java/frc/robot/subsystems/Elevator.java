package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    // TODO: Set soft limits
    // TODO: Limit switches (see getForwardLimitSwitch)

    // Elevator positions in encoder tics
    // TODO: Determine experimentally and then put an equation for offline estimation of changes and put that here for posterity
    public final int RESTING = 0; // TODO make sure powered down completely when resting or very low to avoid power draw
    public final int PROCESSOR = 0;
    public final int LOADING = 0;
    public final int L1 = 0;
    public final int L2 = 0;
    public final int L3 = 0;
    public final int L4 = 0;
    public final int BARGE = 0;

    // TODO: Set CAN ids
    private final int LEADER_CAN_ID = 0;
    private final int FOLLOWER_CAN_ID = 0;

    // Motor controllers: one leader and one follower
    private final SparkMax m_leader;
    private final SparkMax m_follower;

    // The throughbore encoder is on the leader
    private final RelativeEncoder m_output_encoder;

    // REV's built-in PID controller on the leader
    private final SparkClosedLoopController m_sparkClosedLoopController;

    // PID gains TODO: Tune
    private static final double KP = 0.0;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double MIN_OUTPUT = 0.0;
    private static final double MAX_OUTPUT = 0.0;

    /**
     * Constructs the Elevator subsystem.
     */
    public Elevator() {
        // Initialize motor controllers
        m_leader = new SparkMax(LEADER_CAN_ID, MotorType.kBrushless);
        m_follower = new SparkMax(FOLLOWER_CAN_ID, MotorType.kBrushless);

        // Configure the motors

        // TODO Get help / rework the control loop for good control and smoothness
        // Ref https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started
        // and see what we can do to tune and improve the control.

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig.closedLoop
            .p(KP)
            .i(KI)
            .d(KD)
            .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        followerConfig.follow(m_leader, true /* inverted */);

        m_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get the encoder from the leader (the throughbore encoder on the output shaft)
        m_output_encoder = m_leader.getAlternateEncoder();

        // The onboard controller
        m_sparkClosedLoopController = m_leader.getClosedLoopController();
    }

    /**
     * Sets the desired elevator position.
     *
     * @param position The target position
     */
    public void setTargetPosition(double position) {
        double feed_forward = 0.0; // TODO and may be different going down

        m_sparkClosedLoopController.setReference(
            position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0, // TODO
            feed_forward);
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
    }
}
