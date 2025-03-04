// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.GPMConstants.ArmConstants;
import frc.robot.Constants.GPMConstants.ClimberConstants;
import frc.robot.Constants.GPMConstants.IntakeConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;
import frc.robot.commands.TeleopAlgaePickupFromLow;
import frc.robot.commands.TeleopAlgaeSpitOut;
import frc.robot.commands.AlgaeToBargeAndShoot;
import frc.robot.commands.AlgaeToProcessorAndShoot;
import frc.robot.commands.ArmRunWithSpeed;
import frc.robot.commands.ArmToPositionAndHold;
import frc.robot.commands.AutoStraightTrajectoryToReef8;
import frc.robot.commands.AutonomousTrajectory2Poses;
import frc.robot.commands.CalibrateArmMoveManually;
import frc.robot.commands.CalibrateChassisAngularDeadband;
import frc.robot.commands.CalibrateElevatorDeterminekG;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.IntakeAlgaeOutSequence;
import frc.robot.commands.IntakeCoralAndMoveToCruisePositionSequence;
import frc.robot.commands.IntakeCoralOutCommand;
import frc.robot.commands.IntakeShootCommand;
import frc.robot.commands.PanToReefTarget;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.ClimberStartWithSpeed;
import frc.robot.commands.CoralPlaceOnFour;
import frc.robot.commands.CoralPlaceOnOne;
import frc.robot.commands.CoralPlaceOnThree;
import frc.robot.commands.CoralPlaceOnTwo;
import frc.robot.commands.StopArm;
import frc.robot.commands.StopClimber;
import frc.robot.commands.StopElevator;
import frc.robot.commands.StopElevatorAndHold;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TestIntakeCoralPlaceOnFour;
import frc.robot.commands.TestIntakeCoralPlaceOnThree;
import frc.robot.commands.TestIntakeCoralPlaceOnTwo;
import frc.robot.commands.TurnToRelativeAngleTrapezoidProfile;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LLVisionSubsystem;
import frc.robot.subsystems.ReefFinderSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final ReefFinderSubsystem reefFinderSubsystem = new ReefFinderSubsystem();
  public static final LLVisionSubsystem llVisionSubsystem = new LLVisionSubsystem();

  public static Controller xboxDriveController;
  public static Controller xboxGPMController;
  public static Joystick buttonBox;
  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;

  public static Joystick driveStick1;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDriverInterface(); 
    configureBindings();
    //calibrateElevator();
    //calibrateArm();

    driveSubsystem.setDefaultCommand(
      new DriveManuallyCommand(
          () -> getDriverXAxis(),
          () -> getDriverYAxis(),
          () -> getDriverOmegaAxis()));

    if (EnabledSubsystems.arm) {
      armSubsystem.calibrateZeroArmPosition();
    }
  }

  private void configureDriverInterface(){
    driveStick1 = new Joystick(0); //TODO: ONLY FOR TESTING; MUST BE COMMENTED FOR COMP
    xboxDriveController = new Controller(ControllerDevice.XBOX_CONTROLLER);
    xboxGPMController = new Controller(ControllerDevice.XBOX_CONTROLLER_GPM);
    buttonBox = new Joystick(OIConstants.buttonBoxPort);

  }

    // Alliance color determination
  public void checkAllianceColor() {
    SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
  }

  public static void setIfAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (! alliance.isPresent()) {
        System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
    } else {
        isAllianceRed = alliance.get() == DriverStation.Alliance.Red;
        System.out.println("*** RED Alliance: "+isAllianceRed);
    }
  }
  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }

   // Driver preferred controls
   private double getDriverXAxis() {
    return -xboxDriveController.getLeftStickY();
    //return -xboxDriveController.getRightStickY();
  }

  private double getDriverYAxis() {
    return -xboxDriveController.getLeftStickX();
    //return -xboxDriveController.getRightStickX();
  }

  private double getDriverOmegaAxis() {
    //return -xboxController.getLeftStickOmega();
    //return -xboxDriveController.getLeftStickX();
    return -xboxDriveController.getRightStickX();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    try {
      //testAuto();
      //testElevator();
      //testMohawk();
      //testAutoChoate();
      //competitionButtonBoxBinding();
    }
    catch (Exception e) {
       System.out.println("test auto error: " + e);
    }

    // testTurn();
    setYaws();
    //testIntake();
    //testArm(); 
       //testVisionCoordoinates();
    //calibrateElevator(); 
    
   
  }


  public void competitionButtonBoxBinding() {
    new JoystickButton(buttonBox, 1)
      .onTrue(new AlgaeToBargeAndShoot());

    new JoystickButton(buttonBox, 2)
      .onTrue(new AlgaeToProcessorAndShoot());
    
    new JoystickButton(buttonBox, 3)
      .onTrue(new ArmToPositionAndHold(ArmPositions.AlgaeRelease).andThen(new IntakeAlgaeOutSequence()));
    
    new JoystickButton(buttonBox, 4)
      .onTrue(new CoralPlaceOnFour());

    new JoystickButton(buttonBox, 6)
      .onTrue(new CoralPlaceOnThree());

    new JoystickButton(buttonBox, 8)
      .onTrue(new CoralPlaceOnTwo());

    new JoystickButton(buttonBox, 10)
      .onTrue(new CoralPlaceOnOne());

    new JoystickButton(buttonBox, 12)
      .onTrue(new IntakeCoralOutCommand(0.2)); //TODO: Needs testing
  }

  public void setYaws() {
    new JoystickButton(xboxDriveController, 8)
      .onTrue(new InstantCommand(() -> driveSubsystem.zeroYaw()));
  }

  public void testAuto() throws Exception {
    // new JoystickButton(xboxDriveController, 1)
    //   .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeter90"))
    //   .onFalse(new StopRobot());
    new JoystickButton(driveStick1, 11)
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward"))
      .onFalse(new StopRobot());
  }

  public void testAutoChoate() throws Exception {

    System.out.println("Def CH");
    new JoystickButton(driveStick1, 12)
      .onTrue(new AutoStraightTrajectoryToReef8())
      .onFalse(new StopRobot());
      System.out.println("End Def CH");
  }

  public void testReef() throws Exception {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new PanToReefTarget(0.05*SwerveChassis.MaxSpeed))
      .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 2)
      .onTrue(new PanToReefTarget(-0.05*SwerveChassis.MaxSpeed))
      .onFalse(new StopRobot());
  }

  public void testArm() {
    // new JoystickButton(xboxDriveController, 1) 
    //   .onTrue(new ArmRunWithSpeed(0.2))
    //   .onFalse(new StopArm());

    // new JoystickButton(xboxDriveController, 1)
    //   .onTrue(new ArmToPositionAndHold(ArmConstants.ArmPositions.ReefLevelFour))
    //   .onFalse(new StopArm());

    // new JoystickButton(xboxDriveController, 2)
    //   .onTrue(new TestArmToPosition(0))
    //   .onFalse(new StopArm());

    // new JoystickButton(xboxDriveController, 3)
    //   .onTrue(new ArmToPositionAndHold(ArmConstants.ArmPositions.CoralIntake))
    //   .onFalse(new StopArm());

    // new JoystickButton(driveStick1, 11)
    // .onTrue(new StopArm());

    // new JoystickButton(driveStick1, 12)
    //   .onTrue(new ArmToPositionAndHold(ArmConstants.ArmPositions.AlgaeIntake));

    // new JoystickButton(driveStick1, 9)
    //   .onTrue(new ArmToPositionAndHold(ArmConstants.ArmPositions.AlgaeRelease));

    // new JoystickButton(driveStick1, 7)
    //   .onTrue(new IntakeAlgaeRollOutCommand());

    // new JoystickButton(driveStick1, 8)
    //   .onTrue(new StopIntake());

    // new JoystickButton(driveStick1, 10)
    //   .onTrue(new IntakeAlgaeRollerInAndHold());

    // new JoystickButton(driveStick1, 6)
    //   .onTrue(new ArmToPositionAndHold(ArmConstants.ArmPositions.CoralCruise));

    // new JoystickButton(driveStick1, 4)
    //   .onTrue(new AlgaePickupReadyFromHigh());

    // new JoystickButton(driveStick1, 3)
    //   .onTrue(new AlgaePickupReadyFromLow());   
      
    // new JoystickButton(driveStick1, 5)
    // .onTrue(new IntakeCoralAndMoveToCruisePositionSequence());   
  }

  public void testElevator() throws Exception {
    // new JoystickButton(xboxDriveController, 1)
    //   .onTrue(new ElevatorRunWithSpeed(0.2))
    //   .onFalse(new StopElevator());

    // new JoystickButton(driveStick1, 3)
    //   .onTrue(new ElevatorToLevelAndHold(ElevatorHeights.ReefLevelTwo))
    //   .onFalse(new StopElevator());

    // new JoystickButton(driveStick1, 4)
    //   .onTrue(new ElevatorToLevelAndHold(ElevatorHeights.ReefLevelThree))
    //   .onFalse(new StopElevator());

    // new JoystickButton(driveStick1, 5)
    //   .onTrue(new ElevatorToLevelAndHold(ElevatorHeights.ReefLevelFour))
    //   .onFalse(new StopElevator());

    // new JoystickButton(driveStick1, 10)
    //   .onTrue(new TestIntakeCoralPlaceOnTwo())
    //   .onFalse(new StopElevator());

    // new JoystickButton(driveStick1, 8)
    //   .onTrue(new TestIntakeCoralPlaceOnThree())
    //   .onFalse(new StopElevator());

    // new JoystickButton(driveStick1, 7)
    //   .onTrue(new TestIntakeCoralPlaceOnFour())
    //   .onFalse(new StopElevator());

    // new JoystickButton(driveStick1, 7)
    //   .onTrue(new TestAllCoralLevelsCommand())
    //   .onFalse(new StopElevator());


    new JoystickButton(driveStick1, 7)
      .onTrue(
        new ArmToPositionAndHold(ArmPositions.CoralCruise)
        .andThen(new WaitCommand(1))
        .andThen(new ArmToPositionAndHold(ArmPositions.ReefLevelFour))
        .andThen(new WaitCommand(1))
        .andThen(new ArmToPositionAndHold(ArmPositions.CoralCruise)) 
      )
      .onFalse(new StopArm());

  }

  public void testClimber() throws Exception {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new ClimberStartWithSpeed(0.2))
      .onFalse(new StopClimber());
  }


  public void calibrateElevator() {
    new JoystickButton(driveStick1, 1)
    .onTrue(new CalibrateElevatorDeterminekG())
    .onFalse(new StopElevatorAndHold());

    new JoystickButton(driveStick1, 2)
    .onTrue(new StopElevator());
  }

  public void calibrateArm() {
    new JoystickButton(xboxDriveController, 2)
    .onTrue(new CalibrateArmMoveManually())
    .onFalse(new StopArm());
  }

  public void testTurn() {
    // new JoystickButton(xboxDriveController, 3)
    // .onTrue(new InstantCommand(() -> driveSubsystem.drive(0, 0, driveSubsystem.getChassisAngularVelocityConversion(0.5))))
    // //.onTrue(new PrintCommand("V: " + driveSubsystem.getChassisAngularVelocityConversion(2.0)))
    // .onFalse(new InstantCommand(() -> driveSubsystem.stopRobot()));

    new JoystickButton(xboxDriveController, 3)
      .onTrue(new TurnToRelativeAngleTrapezoidProfile(35, () -> driveSubsystem.getYaw()))
      .onFalse(new StopRobot());

    // new JoystickButton(xboxDriveController, 3)
    //    .onTrue(new CalibrateDriveTrainTurnMinimumPower())
    //    .onFalse(new StopRobot());
  }

  public void testIntake() {
    new JoystickButton(xboxDriveController, 5)
    .onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(0.5),intakeSubsystem))
    .onFalse(new StopIntake());

    new JoystickButton(xboxDriveController, 6)
    .onTrue(new IntakeCoralAndMoveToCruisePositionSequence());

    // new JoystickButton(driveStick1, 11)
    // .onTrue(new StopArm());

    // new JoystickButton(xboxDriveController, 3)
    // .onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(-0.5),intakeSubsystem))
    // .onFalse(new StopIntake());

    // new JoystickButton(xboxDriveController, 4)
    // .onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(1.0),intakeSubsystem))
    // .onFalse(new StopIntake());

    // new JoystickButton(xboxDriveController, 5)
    // .onTrue(new InstantCommand(() -> intakeSubsystem.runIntake(-0.2),intakeSubsystem))
    // .onFalse(new StopIntake());

    // new JoystickButton(xboxDriveController, 6)
    //   .onTrue(new IntakeAlgaeInCommand())
    //   .onFalse(new IntakeAlgaeOutSequence());
  }

  public void testChoate() throws Exception{
    new JoystickButton(driveStick1, 1)
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("Blu-BargeToReef8"))
      .onFalse(new StopRobot());
  }

  public void testMohawk() throws Exception { //also some trajectory testing
    // new JoystickButton(driveStick1, 12)
    //    .onTrue(new AlgaePickupReadyFromHigh());

    // new JoystickButton(driveStick1, 11)
    //    .onTrue(new AlgaePickupReadyFromLow());

    // new JoystickButton(driveStick1, 10)
    //    .onTrue(new AlgaeSpitOut())
    //    .onFalse(new StopArm());
    
    new JoystickButton(xboxDriveController, 5)
       .onTrue(new IntakeCoralAndMoveToCruisePositionSequence());

    new JoystickButton(driveStick1, 8)
       .onTrue(new CoralPlaceOnTwo())
       .onFalse(new StopArm());

    new JoystickButton(driveStick1, 7)
       .onTrue(new CoralPlaceOnThree())
       .onFalse(new StopArm());

    new JoystickButton(driveStick1, 6)
       .onTrue(new CoralPlaceOnFour());

    // new JoystickButton(driveStick1, 5)
    //    .onTrue(new PanToReefTarget(0.1))
    //    .onFalse(new StopArm());
    
    // new JoystickButton(driveStick1, 4)
    //    .onTrue(new StopArm());
    
    // new JoystickButton(driveStick1, 3)
    //    .onTrue(new CoralIntakeReadySequence());

    // new JoystickButton(driveStick1,11 )
    //   .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward"))
    //   .onFalse(new StopRobot());

    // new JoystickButton(driveStick1,12 )
    //   .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward-90Turn"))
    //   .onFalse(new StopRobot());
    
    new JoystickButton(driveStick1,12 )
      .onTrue(new AutonomousTrajectory2Poses(new Pose2d(1, 1, Rotation2d.fromDegrees(0)), new Pose2d(2.39, 1, Rotation2d.fromDegrees(0)))
        .andThen(new CoralPlaceOnFour()))
      .onFalse(new StopRobot());

    new JoystickButton(driveStick1, 3)
      .onTrue(new StopElevator().alongWith(new StopIntake()));

    new JoystickButton(driveStick1, 4)
      .onTrue(new IntakeShootCommand());

    // new JoystickButton(driveStick1,12 )
    //    .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward-90Turn"))
    //    .onFalse(new StopRobot());

    // new JoystickButton(driveStick1, 11)
    //   .onTrue(new IntakeCoralOutCommand(IntakeConstants.coralShootingPowerL23)); 

    // new JoystickButton(driveStick1, 1)
    //   .onTrue(new ClimberStartWithSpeed(ClimberConstants.climbUpPower));

  }

  // public void testChoate(){
  //   new JoystickButton(driveStick1, 10)
  //     .onTrue(new CoralPlaceOnFour())
  //     .onFalse(new StopArm());
  // }

  public void buttonBoxBindings() {
    new JoystickButton(buttonBox, 3)
      .onTrue(new TeleopAlgaeSpitOut())
      .onFalse(new StopArm());

    new JoystickButton(buttonBox, 4)
      .onTrue(new TestIntakeCoralPlaceOnFour()); 

    new JoystickButton(buttonBox, 6)
      .onTrue(new TestIntakeCoralPlaceOnThree());

    new JoystickButton(buttonBox, 8)
      .onTrue(new TestIntakeCoralPlaceOnTwo());
  }

  public void calibrateChassisDeadband() {
    // new JoystickButton(driveStick1, 1)
    //   .onTrue(new CalibrateChassisLinearDeadband())
    //   .onFalse(new StopRobot());
    new JoystickButton(driveStick1, 1)
       .onTrue(new CalibrateChassisAngularDeadband())
       .onFalse(new StopRobot());
  }

  public void testVisionCoordoinates() {
    System.out.println("****Poses:  ");
    // System.out.println(llVisionSubsystem.getKnownPose("RobotBluReef1Left"));
    // System.out.println(llVisionSubsystem.getKnownPose("RobotBluReef1Right"));
    
      List<String> keys = new ArrayList<>();
        for(String k : RobotPoseConstants.visionRobotPoses.keySet()) {
            keys.add(k);
        }
        for (String key : keys) { 
          System.out.println(key + RobotPoseConstants.visionRobotPoses.get(key));
        }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return null; 
  }
}
