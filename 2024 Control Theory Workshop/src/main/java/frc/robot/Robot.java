// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private ShuffleboardTab shooterPID = Shuffleboard.getTab("Shooter");
  private GenericEntry kP = shooterPID.add("kP", 0.00002).getEntry();
  private GenericEntry kD = shooterPID.add("kD", 0.00).getEntry();
  private ShuffleboardTab SetpointTab = Shuffleboard.getTab("Shooter");
  private GenericEntry setpoint = SetpointTab.add("setpoint", 1200).getEntry();

  private final XboxController controller = new XboxController(0);
  private final VictorSP rightDrive = new VictorSP(0);
  private final VictorSP leftDrive = new VictorSP(1);
  private final CANSparkMax indexer = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax flywheelA = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax flywheelB = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax hood = new CANSparkMax(8, MotorType.kBrushless);
  private final RelativeEncoder flywheelEncoder = flywheelB.getEncoder();
  private final RelativeEncoder hoodEncoder = hood.getEncoder();

  private final DifferentialDrive robotDrive = new DifferentialDrive(leftDrive::set, rightDrive::set);

  private final PIDController flywheelPID = new PIDController(0.001, 0, 0);
  private double flywheelMoveRequest;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftDrive.setInverted(true);
    indexer.setInverted(true);
    flywheelB.follow(flywheelA, true);

    hood.setIdleMode(IdleMode.kCoast);
    hoodEncoder.setPosition(0);

    flywheelPID.setTolerance(10);
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
    SmartDashboard.putNumber("Flywheel Velocity", flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    hood.setIdleMode(IdleMode.kBrake);
    indexer.setIdleMode(IdleMode.kBrake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotDrive.arcadeDrive(-controller.getLeftY(), -controller.getLeftX());

    if (controller.getRightBumper()) {
      flywheelPID.setPID(kP.getDouble(0), 0, kD.getDouble(0));
      flywheelMoveRequest = setpoint.getDouble(0)/5800+flywheelPID.calculate(flywheelEncoder.getVelocity(), setpoint.getDouble(0));
      flywheelA.set(flywheelMoveRequest);
      SmartDashboard.putNumber("Move Request", flywheelMoveRequest);

      if (flywheelEncoder.getVelocity()>setpoint.getDouble(0)-100) indexer.set(0.5);
      else indexer.set(0);
    }
    else {
      flywheelA.set(0);
      indexer.set(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    hood.setIdleMode(IdleMode.kCoast);
    indexer.setIdleMode(IdleMode.kCoast);
  }

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
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
