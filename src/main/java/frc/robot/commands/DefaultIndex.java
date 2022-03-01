// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class DefaultIndex extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Index subsystem;
  private final DoubleSupplier rotationSupplier;
  private final double motorRevolutions = 1;
  
  private boolean ballIndexed = false;
  private double ballsIndexed = 0;
  private double ballToShoot = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultIndex(Index subsystem, DoubleSupplier rotationSupplier) {
    this.subsystem = subsystem;
    this.rotationSupplier = rotationSupplier;
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
    double rotationSpeed = rotationSupplier.getAsDouble();
    double position =  subsystem.encoder.getPosition();
    subsystem.setReference(position + rotationSpeed);
    if(!ballIndexed && ballsIndexed <= 2){
      if(!subsystem.beambreak.get()){//if a ball breaks the beam, ballIndexed is set to true showing a ball has entered the robot and the ball is moved up
        subsystem.setReference(position+motorRevolutions);
        ballIndexed = true;
      }
    }
    if(ballIndexed){//if a ball is has entered the robot, the total amount of balls indexed is increased
      ballsIndexed += 1;
      ballIndexed = false;
    }
    if(ballToShoot > 0){
      subsystem.setReference(ballToShoot*5);
      //shooter.motor.set(shooter.); to sh
      ballToShoot -= 1;
      ballsIndexed -= 1;
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
