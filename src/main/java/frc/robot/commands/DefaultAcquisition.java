package frc.robot.commands;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquisition;

/** An example command that uses an example subsystem. */
public class DefaultAcquisition extends CommandBase {
  private final Acquisition subsystem;
  private double setpointVelocity = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultAcquisition(Acquisition subsystem) {
    this.subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

 


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("acquisition/setpoint Velocity", setpointVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(subsystem.getArmsExtended()){
      subsystem.motor2.set(setpointVelocity);
      SmartDashboard.putNumber("acquisition/actual Velocity", subsystem.encoder.getVelocity());
      setpointVelocity = SmartDashboard.getNumber("acquisition/setpoint Velocity", setpointVelocity);
      subsystem.pid.setReference(setpointVelocity, ControlType.kVelocity);
    }
    else{
      subsystem.motor2.set(0);
    }
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