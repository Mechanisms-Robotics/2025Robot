package frc.robot.subsystems.algaemech;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeMech extends SubsystemBase {
    private static final SparkMax wristMotor = new SparkMax(0, MotorType.kBrushed);
    private static final SparkMax motorL = new SparkMax(10, MotorType.kBrushed);
    private static final SparkMax motorR = new SparkMax(12, MotorType.kBrushed);
    private static final CANcoder canCoder = new CANcoder(12);

    enum State {
      INTAKING,
      IDLE
    }

    State state = State.INTAKING;
    /**
     * Class containing all the variables that can be tunned in smartdashboard for Algae Mech
     */
    private class Tunnable implements Sendable {
        private double intakeVoltage = 5;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Tunnable Variables");
            builder.addDoubleProperty("intake voltage", () -> intakeVoltage, (k) -> intakeVoltage = k);
        }   
    }

    private final Tunnable tunes;

    public AlgaeMech() {
        tunes = new Tunnable();
        SmartDashboard.putData("AlgaeMech/Tunnable", tunes);

        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kBrake);

        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig wheelMotorConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kBrake);

        motorL.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void intake() {

        motorL.setVoltage(tunes.intakeVoltage);
        motorR.setVoltage(-tunes.intakeVoltage);
        state = State.INTAKING;
        
    }

    public void stop() {
      motorL.setVoltage(0);
      motorR.setVoltage(0);
      state = State.IDLE;
    }

    public void toggleIntake() {
      if (state == State.INTAKING) {
        stop();
      } else {
        intake();
      }
    }

    @Override
    public void periodic() {
        // ranges from -.5-.5
        SmartDashboard.putNumber("AlgaeMech/Wrist/absolute position", canCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/Wrist/position", canCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/Wrist/position since boot", canCoder.getPositionSinceBoot().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/Wrist/angle", canCoder.getPositionSinceBoot().getValueAsDouble()%.5 * 2 * 360);
        SmartDashboard.putNumber("AlgaeMech/Left Wheel Motor/Applied Output", motorL.getAppliedOutput());
        SmartDashboard.putNumber("AlgaeMech/Left Wheel Motor/Applied Output", motorL.getAppliedOutput());
        SmartDashboard.putNumber("AlgaeMech/Left Wheel Motor/id", motorL.getDeviceId());
        SmartDashboard.putString("AlgaeMech/State", state.toString());
    }
}
