package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    SparkMax elevatorMotor = new SparkMax(9, SparkMax.MotorType.kBrushless);
    CANcoder elevatorEncoder = new CANcoder(10);
    PIDController controller = new PIDController(0.01, 0.0, 0.0);
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.5, 0.0, 0.0);

    enum State {
        GROUND,
        L1,
        L2,
        L3,
        L4
    }

    State state = State.GROUND;

    // TODO: figure these values out
    // max ticks is used to convert inches to ticks, the relationship between maxTicks and max Height is linear
    private final double maxTicks = 100;
    private final double offsetTicks = 0;
    private final double maxHeight = 92; // inches
    private final double l1Ticks = inchesToTicks(10);
    private final double l2Ticks = inchesToTicks(20);
    private final double l3Ticks = inchesToTicks(30); 
    private final double l4Ticks = inchesToTicks(40);

    private final Alert badLevelArguent = new Alert("Level argument invalid, must be 0-4", Alert.AlertType.kError);

    public void setLevel(int num) {
        System.out.println("bing bong");
        badLevelArguent.set(true);
        switch (num) {
            case 0:
                controller.setSetpoint(0);
                state = State.GROUND;
                break;
            case 1:
                controller.setSetpoint(l1Ticks);
                state = State.L1;
                break;
            case 2:
                controller.setSetpoint(l2Ticks);
                state = State.L2;
                break;
            case 3:
                controller.setSetpoint(l3Ticks);
                state = State.L3;
                break;
            case 4:
                controller.setSetpoint(l4Ticks);
                state = State.L4;
                break;

        }
    }

    private double inchesToTicks(double inches) {
        return maxTicks / maxHeight * inches - offsetTicks;
    }

    @Override
    public void periodic() {
        double output = feedforward.calculate(elevatorEncoder.getVelocity().getValueAsDouble());
        elevatorMotor.set(output);

        SmartDashboard.putNumber("Elevator State", state.ordinal());
    }
}
