// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoArmToL4AndHold extends Command {
  /** Creates a new ArmToPositionAndHold. */
  boolean safeToMove = false;

  private ArmPositions setPosition;

  /**
   * Move arm to position described in the ArmPosition ENUM via PID and hold (the command will not end PID)
   * @param position - ArmPositon ENUM with values (rotations)
   */
  public AutoArmToL4AndHold(ArmPositions position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem);
    setPosition = position;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Arm going to position: " + setPosition);
    safeToMove = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!safeToMove && RobotContainer.elevatorSubsystem.getElevatorHeight() > ElevatorHeights.ArmTurnSafe.getHeight()) {
      safeToMove = true;
      RobotContainer.armSubsystem.setArmPositionWithAngle(setPosition);
    }
    System.out.println(safeToMove);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm at position: " + setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.armSubsystem.isAtPosition(setPosition);
  }
}
