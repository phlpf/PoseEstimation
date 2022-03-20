package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquisition;

/** An example command that uses an example subsystem. */
public class DefaultAcquisition extends CommandBase {
  private final Acquisition acquisition;

  public DefaultAcquisition(Acquisition acquisition) {
    this.acquisition = acquisition;
    addRequirements(acquisition);
  }


  // Called every time the scheduler runs while the command is scheduled.
  
  public void execute(){
    if (acquisition.getSetpointRPM() == 0) {
      acquisition.stopRollersByVoltage();
      acquisition.retractArms();
    } else {
      acquisition.extendArms();
      acquisition.runClosedLoopRPM();
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