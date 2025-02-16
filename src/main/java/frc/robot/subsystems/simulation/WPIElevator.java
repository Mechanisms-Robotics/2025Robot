// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.ElevatorFeedforward;

public class WPIElevator implements AutoCloseable {
  private double p = 0;
  private double i = 0;
  private double d = 0;
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

  private final ExponentialProfile m_profile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(
              Constants.kElevatorMaxV, Constants.kElevatorkV, Constants.kElevatorkA));

  private ExponentialProfile.State m_setpoint = new ExponentialProfile.State(0, 0);

  // Standard classes for controlling our elevator
  private final PIDController m_pidController =
      new PIDController(p, i, d);

  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);
  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          Constants.kElevatorGearing,
          Constants.kCarriageMass,
          Constants.kElevatorDrumRadius,
          Constants.kMinElevatorHeightMeters,
          Constants.kMaxElevatorHeightMeters,
          true,
          0,
          0.005,
          0.0);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d =
      new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(100));
  private final MechanismRoot2d m_mech2dRoot =
      m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(5), Units.inchesToMeters(2));
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /** Subsystem constructor. */
  public WPIElevator() {
    SmartDashboard.putNumber("P", p);
    SmartDashboard.putNumber("I", i);
    SmartDashboard.putNumber("D", d);

    m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    SmartDashboard.putNumber("Position", Units.metersToInches(m_elevatorSim.getPositionMeters()));
    SmartDashboard.putNumber("Setpoint", Units.metersToInches(m_pidController.getSetpoint()));
    p = SmartDashboard.getNumber("P", p);
    i = SmartDashboard.getNumber("I", i);
    d = SmartDashboard.getNumber("D", d);

    m_pidController.setP(p);
    m_pidController.setI(i);
    m_pidController.setD(d);

    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    var goalState = new ExponentialProfile.State(goal, 0);

    var next = m_profile.calculate(0.020, m_setpoint, goalState);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_pidController.calculate(m_encoder.getDistance(), m_setpoint.position);
    double feedforwardOutput =
        m_feedforward.calculateWithVelocities(m_setpoint.velocity, next.velocity);

    m_motor.setVoltage(pidOutput + feedforwardOutput);

    m_setpoint = next;
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_motor.set(0.0);
  }

  /** Reset Exponential profile to begin from current position on enable. */
  public void reset() {
    m_setpoint = new ExponentialProfile.State(m_encoder.getDistance(), 0);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
}
