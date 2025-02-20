package frc.robot.subsystems.algaemech;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeMech extends SubsystemBase {
    private static final SparkMax wristMotor = new SparkMax(0, MotorType.kBrushed);
    private static final SparkMax motorL = new SparkMax(1, MotorType.kBrushed);
    private static final SparkMax motorR = new SparkMax(2, MotorType.kBrushed);
    private static final CANcoder canCoder = new CANcoder(12);

    private static final double intakeVoltage = 1;

    public AlgaeMech() {
        SmartDashboard.putNumber("AlgaeMech/intake voltage", intakeVoltage);

        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kBrake);

        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig wheelMotorConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kBrake);

        motorL.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void intake() {
        motorL.setVoltage(intakeVoltage);
        motorR.setVoltage(-intakeVoltage);
    }

    @Override
    public void periodic() {
        // ranges from -.5-.5
        SmartDashboard.putNumber("AlgaeMech/absolute position", canCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/position", canCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/position since boot", canCoder.getPositionSinceBoot().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/angle", canCoder.getPositionSinceBoot().getValueAsDouble()%.5 * 2 * 360);
    }
}
