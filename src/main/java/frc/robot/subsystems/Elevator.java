package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;

public class Elevator extends SubsystemBase {
    // hardware
    private final SparkMax motor = new SparkMax(9, SparkMax.MotorType.kBrushless);
    private final Encoder encoder = new Encoder(10, 11);
    private final double maxHeight = 92; // inches
    private final double drumRadius = Units.inchesToMeters(1);
    
    // controllers
    private final ExponentialProfile profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(10, 5, 0));
    private final ExponentialProfile.State setpoint = new ExponentialProfile.State(0, 0);
    private final PIDController controller = new PIDController(0.01, 0.0, 0.0);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.5, 0.0, 0.0);

    // simulator attributes
    private final DCMotor gearbox = DCMotor.getNeoVortex(2);
    private final double gearing = 10.0;
    private final ElevatorSim elevatorSim = new ElevatorSim(gearbox, gearing, 1, Units.inchesToMeters(2), Units.inchesToMeters(36), Units.inchesToMeters(92), true, 0.0);
    private final SparkMaxSim motorSim = new SparkMaxSim(motor, gearbox);
    private final EncoderSim encoderSim = new EncoderSim(encoder);

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d mech2d = new Mechanism2d(10, 100);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 10);
    private final MechanismLigament2d elevatorMech2d =
    mech2dRoot.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

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
    private final double l1Ticks = inchesToTicks(10);
    private final double l2Ticks = inchesToTicks(20);
    private final double l3Ticks = inchesToTicks(30); 
    private final double l4Ticks = inchesToTicks(40);

    private final Alert badLevelArguent = new Alert("Level argument invalid, must be 0-4", Alert.AlertType.kError);

    public Elevator() {
        // SmartDashboard.putData("Elevator Sim", mech2d);
        encoder.setDistancePerPulse(2.0 * Math.PI * drumRadius / 4096);
    }

    public void setLevel(int num) {
        badLevelArguent.set(num > 4 || num < 0);
        switch (num) {
            case 0:
                controller.setSetpoint(offsetTicks);
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
        // double output = feedforward.calculate(encoder.get   .getValueAsDouble());
        // motor.set(output);

        elevatorMech2d.setLength(encoder.getDistance());

        SmartDashboard.putNumber("Elevator State", state.ordinal());
    }
    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motorSim.getVelocity() * RobotController.getBatteryVoltage());
        elevatorSim.update(0.02);

        encoderSim.setDistance(elevatorSim.getPositionMeters());
        
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps())
        );
    }
}
