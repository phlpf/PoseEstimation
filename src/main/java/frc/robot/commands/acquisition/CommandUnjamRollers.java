// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// bring down arms, reverse rollers, bring up arms or change direction of rpllers to original
package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquisition;

public class CommandUnjamRollers extends CommandBase {
  private Acquisition acquisition;

  /** Creates a new CommandUnjamRollers. */
  public CommandUnjamRollers() {
  }

public CommandUnjamRollers(Acquisition acquisition) {
  this.acquisition = acquisition;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  double currentRollerRPM = acquisition.getSetpointRPM();

  @Override
  public void execute() {
    acquisition.extendArms();
    acquisition.setRollerRPM(-3800);
    try {
      Thread.sleep(3000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    } 
    acquisition.stopRollersByVoltage();
    acquisition.setRollerRPM(currentRollerRPM);
    acquisition.retractArms();
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    } 
    acquisition.extendArms();
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
