// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CommandMoveAngle.CurrentLimit;
import frc.robot.constants.kClimb;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.InstantCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ComplexInitializeClimb extends SequentialCommandGroup {
  /** Creates a new ComplexInitializeArm. */
  public ComplexInitializeClimb(Climber climber) {
    addRequirements(climber);
    addCommands(
      new InstantCommand(() -> climber.startInitialize()),
      new ParallelCommandGroup(
       new CommandMoveReach(climber.outerArm, 4, false, kClimb.INNER_NOLOAD_STALL_CURRENT_REACH),
       new CommandMoveReach(climber.innerArm, 4, false, kClimb.INNER_NOLOAD_STALL_CURRENT_REACH)
      ),
      //new WaitCommand(1),
      new ParallelCommandGroup(
        new CommandMoveReach(climber.outerArm, -35, false, kClimb.INNER_NOLOAD_STALL_CURRENT_REACH),
        new CommandMoveReach(climber.innerArm, -35, false, kClimb.INNER_NOLOAD_STALL_CURRENT_REACH)
      ),  
      new ParallelCommandGroup(
        new CommandMoveAngle(climber.outerArm, -100, kClimb.INNER_NOLOAD_STALL_CURRENT_ANGLE),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new CommandMoveAngle(climber.innerArm, -100, kClimb.INNER_NOLOAD_STALL_CURRENT_ANGLE)
        )    
      ),
      new WaitCommand(0.5),
      new InstantCommand(() ->  climber.zeroAngleEncoders()),
      new ParallelCommandGroup(
        new CommandMoveAngle(climber.outerArm, 29, CurrentLimit.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_EXACT),
        new CommandMoveAngle(climber.innerArm, 29, CurrentLimit.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_EXACT)
      ),
      new InstantCommand(() ->  climber.endInitialize())
    );

  }
}

