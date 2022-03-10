// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

/** An example command that uses an example subsystem. */
public class DefaultIndex extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Index index;
  private final double minIndexIncrement = 0.5;
  private boolean ballWasBreakingSensor;
  
  
  public DefaultIndex(Index index) {
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //🤔
    ballWasBreakingSensor = index.isBallBlockingBeam();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ballIsBreakingSensor = index.isBallBlockingBeam();
    if(!ballIsBreakingSensor && ballWasBreakingSensor){
      index.setBallsIndexed(index.getBallsIndexed()+1);
    }

    if(ballIsBreakingSensor && index.getBallsIndexed() < 1){
      index.runClosedLoopPosition(index.getIndexPosition() + minIndexIncrement);
    }
    ballWasBreakingSensor = ballIsBreakingSensor;
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
