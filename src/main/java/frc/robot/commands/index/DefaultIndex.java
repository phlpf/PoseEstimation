// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.kLED;
import frc.robot.subsystems.Index;

/** An example command that uses an example subsystem. */
public class DefaultIndex extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Index index;
  private final double minIndexIncrement = 10;
  private boolean ballWasBreakingSensor;
  private boolean shiftingBall = false;
  
  
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
    int ballsIndexed = index.getBallsIndexed();
    if(ballsIndexed == 0){
      index.runPercentOut(0.1);
    } else {
      index.runPercentOut(0);
    }

    if(ballWasBreakingSensor && !ballIsBreakingSensor){
      shiftingBall = false;
    }

    if(!ballWasBreakingSensor && ballIsBreakingSensor  && ballsIndexed == 1){
      index.setBallsIndexed(ballsIndexed+1);

      switch(index.getBallsIndexed()) {
        case 0:
          Robot.setLED(kLED.BALLS_INDEXED_ZERO);
          break;
        case 1:
          Robot.setLED(kLED.BALLS_INDEXED_ONE);
          break;
        case 2:
          Robot.setLED(kLED.BALLS_INDEXED_TWO);
      }
    }

    if(ballIsBreakingSensor && ballsIndexed == 0){
      index.setBallsIndexed(ballsIndexed+1);
      shiftingBall = true;
    }

    if(shiftingBall){
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
