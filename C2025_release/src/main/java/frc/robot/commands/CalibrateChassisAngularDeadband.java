// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateChassisAngularDeadband extends Command {
  /** Creates a new CalibrateChassisLinearDeadband. */
  public CalibrateChassisAngularDeadband() {
    addRequirements(RobotContainer.driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveSubsystem.drive(0, 0, SwerveChassis.MaxAngularRate * RobotContainer.driveStick1.getRawAxis(3) * 0.1);
    SmartDashboard.putNumber("C J: ", RobotContainer.driveStick1.getRawAxis(3) * 0.1);
    SmartDashboard.putNumber("C V: ", RobotContainer.driveStick1.getRawAxis(3) * 0.1 * SwerveChassis.MaxAngularRate);
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
