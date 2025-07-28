// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GPMConstants.IntakeConstants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralPanToFinal extends Command {
  int counter = 0;
  int intakeState = 0; // 0 - taking coral IN, 1 - rolling it back when too far

  /** Creates a new IntakeCoralPanToFinal. */
  public IntakeCoralPanToFinal() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    intakeState = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.stopIntake();
    counter = 0;
    System.out.println("*** IntakeCoral command ended " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (intakeState) {
      case 0:
        if (RobotContainer.intakeSubsystem.isTargetVisible() &&
            counter == 0 && RobotContainer.intakeSubsystem.getDistanceToTarget() < 0.2) {
          RobotContainer.intakeSubsystem.runIntake(0.05);
          counter++;
        }
        // if (counter != 0) {
        //   counter++;
        // }

        // State change check
        if (RobotContainer.intakeSubsystem.isPostIntakeTargetVisible()) {
          intakeState = 1;
          RobotContainer.intakeSubsystem.runIntake(IntakeConstants.coralReversePower); // start reversing the coral
          //return false; // continue with reversal
        }

        //return counter > 0;
        return false;
      case 1:
        return ! RobotContainer.intakeSubsystem.isPostIntakeTargetVisible();
    }

    return false;
  }
}
