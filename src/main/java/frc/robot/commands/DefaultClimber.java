// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class DefaultClimber extends CommandBase {
    /** Creates a new DefaultClimber. */
    private Climber climber;
    private XboxController gamepad;
    public DefaultClimber(Climber climber, XboxController gamepad) {
        this.climber = climber;
        this.gamepad = gamepad;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.innerArm.moveReachPOut(gamepad.getRightY());
        climber.innerArm.moveAnglePOut(gamepad.getRightX());
        climber.outerArm.moveReachPOut(gamepad.getLeftY());
        climber.outerArm.moveAnglePOut(gamepad.getLeftX());
        
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
