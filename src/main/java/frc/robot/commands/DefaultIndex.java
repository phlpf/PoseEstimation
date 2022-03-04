// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

/** An example command that uses an example subsystem. */
public class DefaultIndex extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Index subsystem;
  private final double motorRevolutions = 1;
  private final double shootRotations = 5;

  private boolean ballIndexed = false;
  private double ballsIndexed = 0;
  private double ballToShoot = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultIndex(Index subsystem) {
    this.subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("rotations", subsystem.encoder.getPosition());
   
    SmartDashboard.putNumber("ballToShoot", ballToShoot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double position =  subsystem.encoder.getPosition();

    ballCheck();
    if(ballsIndexed < 1){
      if(!subsystem.beambreak.get()){
        subsystem.setReference(position+motorRevolutions);
      }
    }
  }
  
  public void ballCheck(){
    if(subsystem.beambreak.get() != ballIndexed){
      System.out.print("SENSOR CHANGE!");
      ballsIndexed ++;
    }
    ballIndexed = subsystem.beambreak.get();
  }

  public void shootBall(int ballsToShoot){
    subsystem.setReference(ballToShoot*shootRotations);
    ballsIndexed -= ballToShoot;
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
