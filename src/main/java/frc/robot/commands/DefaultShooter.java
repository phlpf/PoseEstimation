// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

/** An example command that uses an example subsystem. */
public class DefaultShooter extends CommandBase {
  private final Shooter subsystem;
  private double setpointVelocity = 0;
  private BooleanSupplier isOnSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultShooter(Shooter subsystem, BooleanSupplier isOnSupplier ) {
    this.subsystem = subsystem;
    this.isOnSupplier = isOnSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("shooter/setpoint Velocity Shooter", setpointVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isOn = isOnSupplier.getAsBoolean();
    if(isOn){  
      subsystem.motor3.set(setpointVelocity);
      SmartDashboard.putNumber("shooter/actual Velocity Shooter", subsystem.encoder.getVelocity());
      setpointVelocity = SmartDashboard.getNumber("shooter/setpoint Velocity Shooter", setpointVelocity);
      subsystem.pid.setReference(setpointVelocity, ControlType.kVelocity);
    }
    else{
      setpointVelocity = 0;
      subsystem.pid.setReference(setpointVelocity, ControlType.kVelocity);

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
