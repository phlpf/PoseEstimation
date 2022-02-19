package frc.robot.commands;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultAcquisition extends CommandBase {
  private final Acquisition m_subsystem;
  private double setpointVelocity = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultAcquisition(Acquisition subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("setpoint Velocity", setpointVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.motor2.set(setpointVelocity);
    SmartDashboard.putNumber("actual Velocity", m_subsystem.encoder.getVelocity());
    setpointVelocity = SmartDashboard.getNumber("setpoint Velocity", setpointVelocity);
    m_subsystem.pid.setReference(setpointVelocity, ControlType.kVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}