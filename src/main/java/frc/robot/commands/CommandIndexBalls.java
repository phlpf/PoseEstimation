// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class CommandIndexBalls extends CommandBase {
  /** Creates a new CommandIndexBalls. */
  Index index;
  int balls;
  public CommandIndexBalls(Index index, int balls) {
    this.index = index;
    this.balls = balls; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.runClosedLoopPosition(index.getIndexPosition() + balls*10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // TODO: Check position reached
  }
}
