package frc.robot.subsystems.algaemech;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeMech extends SubsystemBase {
    private static final SparkMax wristMotor = new SparkMax(0, MotorType.kBrushed);
    private static final SparkMax motorL = new SparkMax(1, MotorType.kBrushed);
    private static final SparkMax motorR = new SparkMax(2, MotorType.kBrushed);

    private static final double intakeVoltage = 1;

    public AlgaeMech() {
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
}
