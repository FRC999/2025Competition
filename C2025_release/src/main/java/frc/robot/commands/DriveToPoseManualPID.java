// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManualPIDConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseManualPID extends Command {

  private Supplier<Pose2d> targetPoseSupplier;
  private Supplier<Pose2d> robotPoseSupplier; // Supplier of the pose; if vision is used, supplier must accommodate
  private final Consumer<Boolean> onTarget; // At the end of the run inform a caller if we actually got within tolerance where we wanted to go
  private final Consumer<Double> distanceFromReef;
  private Pose2d targetPose; // Where we're going
  private double linearTolerance; // X/Y tolerance in meters
  private double angleTolerance;
  private double timeout;
  private Timer timer;

  private final PIDController xController =
      new PIDController(ManualPIDConstants.linearKp, ManualPIDConstants.linearKi, ManualPIDConstants.linearKd, ManualPIDConstants.PIDUpdateFrequency);
  private final PIDController yController =
      new PIDController(ManualPIDConstants.linearKp, ManualPIDConstants.linearKi, ManualPIDConstants.linearKd, ManualPIDConstants.PIDUpdateFrequency);
  private final PIDController thetaController =
      new PIDController(ManualPIDConstants.angularKp, ManualPIDConstants.angularKi, ManualPIDConstants.angularKd, ManualPIDConstants.PIDUpdateFrequency);

  /**
   * Driving to pose with manual PID -
   * used for the last-leg drive navigation
   */
  public DriveToPoseManualPID(
      Supplier<Pose2d> targetPoseSupplier,  // where we're going
      Supplier<Pose2d> robotPoseSupplier,   // Pose supplier - can be vision-aided
      Consumer<Boolean> onTargetConsumer,   // boolean that can be set at the end - did we actually arrive?
      Consumer<Double> distanceFromReef,    // just tracking how far I am from target plane
      double linearTolerance,
      double angleTolerance,
      double timeout                        // how much maximum time in seconds do I have to get to the target
      ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetPoseSupplier = targetPoseSupplier;
    this.robotPoseSupplier = robotPoseSupplier;
    this.onTarget = onTargetConsumer;
    this.distanceFromReef = distanceFromReef;
    this.linearTolerance = linearTolerance;
    this.angleTolerance = angleTolerance;
    this.timer = new Timer();
    this.timeout = timeout;
    addRequirements(RobotContainer.driveSubsystem);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.targetPose = targetPoseSupplier.get();
    this.timer.restart();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = robotPoseSupplier.get();

    double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());

    Transform2d reefRelativeDifference = new Transform2d(targetPose, currentPose);
    var reefRelativeVelocities =
        new Translation2d(xVelocity, yVelocity).rotateBy(targetPose.getRotation().unaryMinus());
    // add 0.25 to the reef relative x velocity to make sure we run into it
    reefRelativeVelocities =
        new Translation2d(reefRelativeVelocities.getX() + 0.25, reefRelativeVelocities.getY());

    if (Math.abs(reefRelativeDifference.getX()) < 0.0762) {
      if (reefRelativeDifference.getY() > 0) {
        reefRelativeVelocities =
            new Translation2d(
                reefRelativeVelocities.getX(),
                reefRelativeVelocities.getY() - ManualPIDConstants.closeVelocityBoost);
      } else if (reefRelativeDifference.getY() < 0) {
        reefRelativeVelocities =
            new Translation2d(
                reefRelativeVelocities.getX(),
                reefRelativeVelocities.getY() + ManualPIDConstants.closeVelocityBoost);
      }
    }

    var fieldRelativeVelocities = reefRelativeVelocities.rotateBy(targetPose.getRotation());
    int allianceMultiplier = RobotContainer.isAllianceRed ? -1 : 1;

    // Test this first !!!!
    RobotContainer.driveSubsystem.drive(
        allianceMultiplier * fieldRelativeVelocities.getX(),
        allianceMultiplier * fieldRelativeVelocities.getY(),
        thetaVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("*** Manual drive to target is done: "+ interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    Transform2d reefRelativeDifference = new Transform2d(targetPose, robotPoseSupplier.get());
    distanceFromReef.accept(reefRelativeDifference.getX());
    boolean atGoal =
    Math.abs(reefRelativeDifference.getX()) < linearTolerance
        && Math.abs(reefRelativeDifference.getY()) < linearTolerance
        && Math.abs(reefRelativeDifference.getRotation().getDegrees())
            < angleTolerance;

    if (atGoal) {
      onTarget.accept(true);
    } else if (this.timer.hasElapsed(timeout)) {
      onTarget.accept(false);
    }

    return this.timer.hasElapsed(timeout) || atGoal;

  }
}
