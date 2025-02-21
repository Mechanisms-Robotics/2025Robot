// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.algaemech.AlgaeMech;
import frc.robot.subsystems.swerve.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  Swerve swerve = new Swerve();
  AlgaeMech algaeMech = new AlgaeMech();
  Test test = new Test();

  CommandPS4Controller ps4Controller = new CommandPS4Controller(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
     autoChooser.setDefaultOption("Leave", new PathPlannerAuto("Leave"));
    //  autoChooser.addOption("My Auto", kCustomAuto);
     SmartDashboard.putData("Auto choices", autoChooser);

    if (Robot.isSimulation()) {
      swerve.setClosedLoop();
      if (Constants.usingKeyboard) {
        Joystick awsd = new Joystick(1);
        swerve.setClosedLoop();
        swerve.setDefaultCommand(
           new RunCommand(() -> swerve.teleopDrive(
              ()->ps4Controller.getLeftX(),
              ()->-ps4Controller.getLeftY(),
              ()->-awsd.getX()
            ), swerve)
        );
      } else {
        swerve.setDefaultCommand(
           new RunCommand(() -> swerve.teleopDrive(
              ()->ps4Controller.getLeftX(),
              ()->-ps4Controller.getLeftY(),
              ()->ps4Controller.getRightX()
            ), swerve)
        );
      }
        DriverStation.silenceJoystickConnectionWarning(true);
      } else {
        swerve.setDefaultCommand(
           // TODO these arguments are wrong, figure it out
            new RunCommand(() -> swerve.teleopDrive(
               ()->-ps4Controller.getLeftX(),
               ()->-ps4Controller.getLeftY(),
               ()->-ps4Controller.getRightX()
           ), swerve)
        );
      }


    ps4Controller.L1().whileTrue(new RunCommand(swerve::lock, swerve));
    ps4Controller.L2().whileTrue(new RunCommand(swerve::test0Voltage, swerve));
    ps4Controller.R2().whileTrue(new RunCommand(swerve::testHalfPower, swerve));
    ps4Controller.R1().whileTrue(new RunCommand(algaeMech::intake, algaeMech));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Command don't run without this
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    if (autoChooser.getSelected() != null) {
      autoChooser.getSelected().schedule();
      swerve.setClosedLoop();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    /* When is teleop, run the swerve in open loop because we don't do closed loop for driving */
    if (!Robot.isSimulation()) {
      swerve.setOpenLoop();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
