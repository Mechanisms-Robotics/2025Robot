// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final double kElevatorMaxV = 10.0; // volts (V)
  public static final double kElevatorkS = 0.0; // volts (V)
  public static final double kElevatorkG = 0.0; // volts (V)
  public static final double kElevatorkV = .5; // volts (V)
  public static final double kElevatorkA = 0.0; // volts (V)

  public static final double kElevatorGearing = 5.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(1.0);
  public static final double kCarriageMass = Units.lbsToKilograms(15); // kg

  public static final double kSetpointMeters = Units.inchesToMeters(42.875);
  public static final double kLowerkSetpointMeters = Units.inchesToMeters(15);
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public static final double kMinElevatorHeightMeters = Units.inchesToMeters(kCarriageMass);
  public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(92);

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  public static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 4096;
}
