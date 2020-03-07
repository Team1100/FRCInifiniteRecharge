/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//This command spins the spinner motor for a certain amount of time.

package frc.robot.commands.Spinner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TestingDashboard;
import frc.robot.subsystems.Spinner;

public class SpinSpinnerMotorTimed extends CommandBase {
  Timer m_timer;
  double m_period;
  Spinner m_spinner;
  double m_speed;
  /**
   * Creates a new SpinSpinnerMotorTimed.
   */
  public SpinSpinnerMotorTimed(double spinnerTime, double spinnerSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Spinner.getInstance());
    m_timer = new Timer();
    m_spinner = Spinner.getInstance();
    m_period = spinnerTime;
    m_speed = spinnerSpeed;
  }

  public static void registerWithTestingDashboard() {
    Spinner spinner = Spinner.getInstance();
    double period = SmartDashboard.getNumber("SpinneMotorPeriod", 5);
    double speed = SmartDashboard.getNumber("SpinnerMotorSpeed", 0.2);  
    SpinSpinnerMotorTimed cmd = new SpinSpinnerMotorTimed(period, speed);
    TestingDashboard.getInstance().registerCommand(spinner, "Timed", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_spinner.spin(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean timerExpired = m_timer.hasPeriodPassed(m_period);
    return timerExpired;
  }
}
