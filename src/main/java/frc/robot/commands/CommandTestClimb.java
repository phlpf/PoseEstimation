// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandMoveAngle.CurrentLimit;
import frc.robot.constants.kClimb;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandTestClimb extends SequentialCommandGroup {
  /** Creates a new CommandClimb. */
  public CommandTestClimb(Climber climber, XboxController gamepad) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CommandMoveAngle(climber.innerArm, -26, CurrentLimit.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_GENERAL),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      new CommandMoveReach(climber.outerArm, kClimb.CLIMB_MAX_EXTEND-2, true),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      new CommandMoveReach(climber.outerArm, kClimb.CLIMB_MIN_EXTEND+0.5, true),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      new CommandMoveAngle(climber.outerArm, 7, CurrentLimit.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_GENERAL),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      
      new CommandMoveReach(climber.innerArm, kClimb.CLIMB_MAX_EXTEND, true, kClimb.INNER_NOLOAD_STALL_CURRENT_REACH),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      //new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new CommandMoveAngleDebounced(climber.innerArm, 0, CurrentLimit.BOTH, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_GENERAL, kClimb.INNER_NOLOAD_STALL_CURRENT_ANGLE),
          new CommandMoveReach(climber.innerArm, kClimb.CLIMB_MIN_EXTEND+0.5, true, kClimb.INNER_NOLOAD_STALL_CURRENT_REACH),
          new InstantCommand(()-> climber.outerArm.setAngleToCoast()),
          new InstantCommand(()-> climber.outerArm.moveAnglePOut(0)),
          new CommandMoveReach(climber.innerArm, kClimb.CLIMB_MIN_EXTEND+0.5, true)
        ),
      //),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON),
      new InstantCommand(()-> climber.outerArm.setAngleToBrake()),
      
      new CommandMoveAngle(climber.outerArm, -26, CurrentLimit.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_GENERAL),
      new CommandWaitForButton(gamepad, kClimb.CLIMB_BUTTON)
    );
  }
}   


