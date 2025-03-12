// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.lib.VisionHelpers;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  public void updateOdometryTelemetry() {
    for (int i =0; i<4; i++){
      SmartDashboard.putNumber("S"+i+" Drive Encoder", RobotContainer.driveSubsystem.getDriveEncoder(i));
      SmartDashboard.putNumber("S"+i+" Drive Velocity", RobotContainer.driveSubsystem.getDriveVelocity(i));
      SmartDashboard.putNumber("S"+i+" Drive EncoderSI", RobotContainer.driveSubsystem.getDriveEncoderSI(i));
      SmartDashboard.putNumber("S"+i+" Drive VelocitySI", RobotContainer.driveSubsystem.getDriveVelocitySI(i));
      SmartDashboard.putNumber("S"+i+" CANCoder Absolute SI", RobotContainer.driveSubsystem.getCancoderAbsoluteSI(i));
      SmartDashboard.putNumber("S"+i+" CANCoder Relative SI", RobotContainer.driveSubsystem.getCancoderRelativeSI(i));
   }
  }

  public void updateChassisTelemetry() {
    SmartDashboard.putString("Chassis Pose", RobotContainer.driveSubsystem.getPose().toString());
    SmartDashboard.putString("Chassis Speeds", RobotContainer.driveSubsystem.getState().Speeds.toString());
  }
  
  public void updateIMUTelemetry() {
    SmartDashboard.putNumber("IMU Yaw", RobotContainer.driveSubsystem.getYaw());
    SmartDashboard.putNumber("Rotation2D: ", RobotContainer.driveSubsystem.getRotation2d().getRadians()); 
    SmartDashboard.putNumber("Imu Speed: ", RobotContainer.driveSubsystem.getPigeon2Speed());
  }

  public void updateArmTelemetry() {
    SmartDashboard.putNumber("Arm ThroughBore Encoder", RobotContainer.armSubsystem.getArmSparkThroughboreEncoderPosition());
    SmartDashboard.putNumber("Arm Motor Encoder Value", RobotContainer.armSubsystem.getKrakenMotorEncoder());
    SmartDashboard.putNumber("Arm Relative Kraken: ", RobotContainer.armSubsystem.armKrakenEncoderRelative());
  }

  public void updateElevatorTelemetry() {
    SmartDashboard.putNumber("Elevator Motor Encoder Value", RobotContainer.elevatorSubsystem.getMotorEncoder());
    SmartDashboard.putBoolean("Elevator Limit Switch : ", RobotContainer.elevatorSubsystem.isLimitSwitchPressed());
  }

  public void updateClimberTelemetry() {
    SmartDashboard.putNumber("Climber Motor Encoder Value", RobotContainer.climberSubsystem.getMotorEncoder());
  }

  public void updateIntakeTelemetry() {
    SmartDashboard.putNumber("IntakeCANrange Distance: ", RobotContainer.intakeSubsystem.getRange());
    SmartDashboard.putBoolean("IntakeCANRangeTargetVisible: ", RobotContainer.intakeSubsystem.isTargetVisible());
    SmartDashboard.putNumber("Intake Output Current: ", RobotContainer.intakeSubsystem.getOutputCurrent());
  }

  public void updateReefFinderTelemetry() {
    SmartDashboard.putBoolean("Reef Target Visibility: ", RobotContainer.reefFinderSubsystem.isTargetVisible());
    SmartDashboard.putNumber("Distance to Reef Target: ", RobotContainer.reefFinderSubsystem.getDistanceToTarget());
  }

  public void updatePeriemeterFinderTelemetry() {
    try {
      SmartDashboard.putBoolean("PeriemeterFront Target Visibility: ", RobotContainer.perimeterFinderSubsystem.isTargetVisibleFront());
      if ( RobotContainer.perimeterFinderSubsystem.isTargetVisibleFront() ) {
        SmartDashboard.putNumber("PerimeterFront Distance: ", RobotContainer.perimeterFinderSubsystem.getDistanceToTargetFront());
      }
      SmartDashboard.putBoolean("PeriemeterBack Target Visibility: ", RobotContainer.perimeterFinderSubsystem.isTargetVisibleBack());
      if ( RobotContainer.perimeterFinderSubsystem.isTargetVisibleBack() ) {
        SmartDashboard.putNumber("PerimeterBack Distance: ", RobotContainer.perimeterFinderSubsystem.getDistanceToTargetBack());
      }
  } catch (Exception e) {
    // TODO: handle exception
  }
  }

  public void updateLLTelemetry() {
    //System.out.println("LL-0");
    try {

      //System.out.println("LL-1");

      SmartDashboard.putBoolean("LL-Any-Visible", RobotContainer.llVisionSubsystem.isAprilTagVisibleBySomeCamera());
      SmartDashboard.putBoolean("LL4-Visible", RobotContainer.llVisionSubsystem.isAprilTagVisibleByLL4());

      //System.out.println("LL-2");

      if (RobotContainer.llVisionSubsystem.isAprilTagVisibleBySomeCamera()) {
        Pose2d atpose = RobotContainer.llVisionSubsystem.getBestPoseAllCameras();
        SmartDashboard.putString("LL-Any-Pose", atpose.toString());
        SmartDashboard.putNumber("LL-ClosestAT-ID",
            RobotPoseConstants.reefTagPoses.get(VisionHelpers.getClosestReefTagToRobot(atpose)));
        SmartDashboard.putString("LL-ClosestAT-Pose", VisionHelpers.getClosestReefTagToRobot(atpose).toString());
        SmartDashboard.putString("AT-Closest-Freiendly-per-IMU",
            RobotContainer.llVisionSubsystem.getTagPerAllianceAndIMU().toString()
                + " " + RobotPoseConstants.redReefTagPoses
                    .get(RobotContainer.llVisionSubsystem.getTagPerAllianceAndIMU()).toString());
        SmartDashboard.putString("Tag17 p2d", VisionHelpers.getTagPose(17).toPose2d().toString());
        SmartDashboard.putString("Tag18 p2d", VisionHelpers.getTagPose(18).toPose2d().toString());
        SmartDashboard.putString("Tag19 p2d", VisionHelpers.getTagPose(19).toPose2d().toString());
      }
      if (RobotContainer.llVisionSubsystem.isAprilTagVisibleByLL4()) {
        SmartDashboard.putString("LL4-Pose", RobotContainer.llVisionSubsystem.getBestPoseLL4s().toString());
      }
    } catch (Exception e) {
      // TODO: handle exception; not sure if needed in telemetry
      //System.out.println("*** Bad things \n" + e);
    }
  }

  public void updateAllianceSideTelemetry() {
    SmartDashboard.putBoolean("IsAllianceRed: ", RobotContainer.isAllianceRed);
    SmartDashboard.putBoolean("IsRedControlsEnabled: ", RobotContainer.isReversingControllerAndIMUForRed);
  }

  public void updateAllDisplays(){

    // updateAllianceSideTelemetry();

    if (DebugTelemetrySubsystems.imu) {
      updateIMUTelemetry();
    }

    if (DebugTelemetrySubsystems.chasis) {
      updateChassisTelemetry();
    }

    if (DebugTelemetrySubsystems.odometry) {
      updateOdometryTelemetry();
    }

    if (DebugTelemetrySubsystems.arm) {
      updateArmTelemetry();
    }

    if (DebugTelemetrySubsystems.elevator) {
      updateElevatorTelemetry();
    }

    if (DebugTelemetrySubsystems.climber) {
      updateClimberTelemetry();
    }
    if(DebugTelemetrySubsystems.intake) {
      updateIntakeTelemetry();
    }

    if(DebugTelemetrySubsystems.reef) {
      updateReefFinderTelemetry();
    }

    if(DebugTelemetrySubsystems.perimeter) {
      updatePeriemeterFinderTelemetry();
    }

    if(DebugTelemetrySubsystems.ll) {
      updateLLTelemetry();
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
