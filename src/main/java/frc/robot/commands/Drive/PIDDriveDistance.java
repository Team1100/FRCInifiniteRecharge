// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.TestingDashboard;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDDriveDistance extends PIDCommand {
  Encoder m_leftEncoder;
  Encoder m_rightEncoder;
  boolean m_parameterized;
  double m_setpoint;
  double m_at_setpoint_counter;
  static Drive m_drive;
  static final double MAX_SPEED = 0.6;
  static final double MIN_SPEED = 0.35;

  /** Creates a new PIDDriveDistance. */
  public PIDDriveDistance(double setpoint, boolean parameterized) {
    super(
        // The controller that the command will use
        new PIDController(.01, .01, 0),
        // This should return the measurement
        () -> {
          Drive drive = Drive.getInstance();
          Encoder leftEncoder = drive.getLeftEncoder();
          Encoder rightEncoder = drive.getRightEncoder();
          return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
        },
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          // Use the output here
          if (output > MAX_SPEED) {
            output = MAX_SPEED;
          }
          if (output < -MAX_SPEED) {
            output = -MAX_SPEED;
          }
          if (output < MIN_SPEED && output > 0) {
            output = MIN_SPEED;
          }
          if (output > -MIN_SPEED && output < 0) {
            output = -MIN_SPEED;
          }
            
          m_drive.tankDrive(output, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(Drive.getInstance());
    getController().setTolerance(2.4);
    getController().disableContinuousInput();
    m_setpoint = setpoint;
    m_parameterized = parameterized;
    m_drive = Drive.getInstance();
    m_leftEncoder = m_drive.getLeftEncoder();
    m_rightEncoder = m_drive.getRightEncoder();
  }

  public static void registerWithTestingDashboard() {

    Drive drive = Drive.getInstance();
    PIDDriveDistance cmd = new PIDDriveDistance(24,false);
    TestingDashboard.getInstance().registerCommand(drive, "Basic", cmd);
    TestingDashboard.getInstance().registerSendable(drive, "PIDDriveDistance", "DrivePIDController", cmd.getController());
  }

  @Override
  public void initialize() {
    super.initialize();
    if (!m_parameterized) {
      m_setpoint = TestingDashboard.getInstance().getNumber(m_drive, "DistanceToTravelInInches");
    }
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_at_setpoint_counter = 0;
  }

  @Override
  public void execute() {
    if (!m_parameterized) {
      m_setpoint = TestingDashboard.getInstance().getNumber(m_drive, "DistanceToTravelInInches");
    }
      
    m_useOutput.accept(
        m_controller.calculate(m_measurement.getAsDouble(), m_setpoint));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (getController().atSetpoint()) {
      m_at_setpoint_counter++;
    }
    boolean finished = m_at_setpoint_counter > 50;
    return finished;
  }
}
