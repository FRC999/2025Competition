// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.commands.TeleopAlgaePickupFromLow;
import frc.robot.commands.TeleopAlgaeSpitOut;
import frc.robot.commands.TeleopCoralIntakeSequence;
import frc.robot.commands.TeleopEjectCoralBringArmToCruiseElevatorDown;
import frc.robot.commands.TeleopIntakeCoralAlternateSequence;
import frc.robot.commands.TeleopMoveToL1RotateArm;
import frc.robot.commands.TeleopMoveToL2RotateArm;
import frc.robot.commands.TeleopMoveToL3RotateArm;
import frc.robot.commands.TeleopMoveToL4RotateArm;
import frc.robot.commands.TeleopPanReefLeft;
import frc.robot.commands.TeleopPanReefRight;
import frc.robot.commands.TeleopPigeonIMUReset;
import frc.robot.commands.AlgaeToBarge;
import frc.robot.commands.AlgaeToProcessor;
import frc.robot.commands.ArmToPositionAndHold;
import frc.robot.commands.AutoBlu1CoralL1;
import frc.robot.commands.AutoBlu2Coral;
import frc.robot.commands.AutoBlueOneCoral;
import frc.robot.commands.AutoRed1CoralL1;
//import frc.robot.commands.AutoRed1CoralL1;
import frc.robot.commands.AutoRed2Coral;
import frc.robot.commands.AutoRedReverse2Coral;
//import frc.robot.commands.AutoRedFromBlu2Coral;
import frc.robot.commands.CalibrateArmMoveManually;
import frc.robot.commands.CalibrateChassisAngularDeadband;
import frc.robot.commands.CalibrateElevatorDeterminekG;
import frc.robot.commands.ClimberStartSequence;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ElevatorAllTheWayDown;
import frc.robot.commands.IntakeAlgaeOutSequence;
import frc.robot.commands.IntakeAlgaeRollOutBargeCommand;
import frc.robot.commands.IntakeAlgaeRollOutCommand;
import frc.robot.commands.IntakeCoralAndMoveToCruisePositionSequence;
import frc.robot.commands.IntakeCoralOutCommand;
import frc.robot.commands.IntakeShootCommand;
import frc.robot.commands.PanLeftRightToReefTargetRobotCentric;
import frc.robot.commands.PanToReefTarget;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.ClimberStartWithSpeed;
import frc.robot.commands.StopArm;
import frc.robot.commands.StopClimber;
import frc.robot.commands.StopElevator;
import frc.robot.commands.StopElevatorAndHold;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TeleopAlgaePickupFromHighAndHold;
import frc.robot.commands.TurnToRelativeAngleTrapezoidProfile;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LLVisionSubsystem;
import frc.robot.subsystems.PerimeterFinderSubsystem;
import frc.robot.subsystems.ReefFinderSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  public static final PerimeterFinderSubsystem perimeterFinderSubsystem = new PerimeterFinderSubsystem();
  public static final LLVisionSubsystem llVisionSubsystem = new LLVisionSubsystem();

  public static Controller xboxDriveController;
  public static Controller xboxGPMController;
  public static Joystick buttonBox;
  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;

  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static Joystick driveStick1;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // PathPlanner AutoBuilder Test
    NamedCommands.registerCommand("ppTest1", runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-BargeToReef11",true, false));
    NamedCommands.registerCommand("ppTest2", runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef11ToCoralTop",false,false));
    NamedCommands.registerCommand("ppTest3", runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralTopToReef10",false,false));
    NamedCommands.registerCommand("ppTest4", runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralTopToReef9",false,false));
    NamedCommands.registerCommand("ppTest5", runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef10ToCoralTop",false,false));
    NamedCommands.registerCommand("ppTest6", runTrajectoryPathPlannerWithForceResetOfStartingPose("Red-BargetoReef11",true,false));
    NamedCommands.registerCommand("ppTest7", runTrajectoryPathPlannerWithForceResetOfStartingPose("Red-Reef11toCoralBottom",false,false));
    NamedCommands.registerCommand("ppTest8", runTrajectoryPathPlannerWithForceResetOfStartingPose("Red-CoralBottomtoReef10",false,false));


    // Configure the trigger bindings
    configureDriverInterface(); 
    configureBindings();
    calibrateElevator();
    //calibrateArm();

    driveSubsystem.setDefaultCommand(
      new DriveManuallyCommand(
          () -> getDriverXAxis(),
          () -> getDriverYAxis(),
          () -> getDriverOmegaAxis()));

    if (EnabledSubsystems.arm) {
      armSubsystem.calibrateZeroArmPosition();
    }

    AutonomousConfigure();
  }

  private void AutonomousConfigure () {
      //port autonomous routines as commands
    //sets the default option of the SendableChooser to the simplest autonomous command. (from touching the hub, drive until outside the tarmac zone) 
    autoChooser.addOption("BLUE TOP 2Coral", new AutoBlu2Coral());
    autoChooser.addOption("RED Bottom 2Coral", new AutoRed2Coral());
    autoChooser.addOption("BLUE One Coral Auto", new AutoBlueOneCoral());
    autoChooser.addOption("BLUE One Coral Auto L1", new AutoBlu1CoralL1());
    autoChooser.addOption("RED One Coral Auto L1", new AutoRed1CoralL1());
    autoChooser.addOption("RED Top 2Coral", new AutoRedReverse2Coral());
    
    //autoChooser.addOption("RED From  Blu 2 ", new AutoRedFromBlu2Coral());

    SmartDashboard.putData(autoChooser);

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
      
      //tryPPTest();
      //tryPPTestCalibration();
      //tryPPTest2();
    }
    catch (Exception e) {
       System.out.println("test auto error: " + e);
    }

    // testTurn();
    //setYaws();
    //testIntake();
    //testArm(); 
       //testVisionCoordoinates();
    //calibrateElevator(); 
    competitionButtonBoxBinding();
    XBOXControllerCompetitionBinding();
    
   
  }


  public void competitionButtonBoxBinding() {
    new JoystickButton(buttonBox, 1)
        .onTrue(new AlgaeToBarge());

    new JoystickButton(buttonBox, 2)
        .onTrue(new AlgaeToProcessor());

    new JoystickButton(buttonBox, 3)
        .onTrue(new TeleopAlgaeSpitOut());

    new JoystickButton(buttonBox, 4)
        .onTrue(new TeleopMoveToL4RotateArm());
    
    new JoystickButton(buttonBox, 5)
        .onTrue(new TeleopMoveToL4RotateArm());

    new JoystickButton(buttonBox, 6)
        .onTrue(new TeleopMoveToL3RotateArm());

    new JoystickButton(buttonBox, 7)
        .onTrue(new TeleopMoveToL3RotateArm());

    new JoystickButton(buttonBox, 8)
        .onTrue(new TeleopMoveToL2RotateArm());

    new JoystickButton(buttonBox, 9)
        .onTrue(new TeleopMoveToL2RotateArm());

    new JoystickButton(buttonBox, 10)
        .onTrue(new TeleopMoveToL1RotateArm());

    new JoystickButton(buttonBox, 11)
        .onTrue(new TeleopMoveToL1RotateArm());

    new JoystickButton(buttonBox, 12)
        .onTrue(new TeleopEjectCoralBringArmToCruiseElevatorDown()); // TODO: Speed needs to be changed accordingly

    new Trigger(() -> buttonBox.getRawAxis(1) == -1.0) //TODO: Axis value needs to be changed as necessary 
      .onTrue(new StopRobot()); 
    
    // new Trigger(() -> buttonBox.getRawAxis(1) == 1.0) //TODO: Axis value needs to be changed as necessary and the speed needs to set after testing
    //   .onTrue(new ClimberStartWithSpeed(-0.2).raceWith(new WaitCommand(1.0)))
    //   .onFalse(new StopClimber());

    //TODO: CHECK IN THE MORNING - VERY IMPORTANT!!!!!

    // Axis 1 - second toggle switch
    new Trigger(() -> buttonBox.getRawAxis(0) > 0.8 && buttonBox.getRawAxis(1) > 0.8)
      .onTrue(new ClimberStartSequence())
      .onFalse(new StopClimber());

    new Trigger(() -> buttonBox.getRawAxis(1) > 0.8)
      .onTrue(new ElevatorAllTheWayDown());

    new Trigger(() -> buttonBox.getRawAxis(0) < -0.8 )
      .onTrue(new InstantCommand(llVisionSubsystem::ToggleBackLLMode))
      ;


    }

  public void XBOXControllerCompetitionBinding() {
    new JoystickButton(xboxDriveController, 5)
    .onTrue(new TeleopAlgaePickupFromHighAndHold()); 

    new JoystickButton(xboxDriveController, 6)
      .onTrue(new TeleopIntakeCoralAlternateSequence())
      .onFalse(new ArmToPositionAndHold(ArmPositions.CoralCruise));

    new Trigger(() -> xboxDriveController.getRawAxis(3) > 0.3) //RT
      .onTrue(new TeleopCoralIntakeSequence())
      .onFalse(new ArmToPositionAndHold(ArmPositions.CoralCruise));

    new Trigger(() -> xboxDriveController.getRawAxis(2) > 0.3) // LT
        .onTrue(new TeleopAlgaePickupFromLow());

    new JoystickButton(xboxDriveController, 8)
        .onTrue(new TeleopPigeonIMUReset());
    
    new Trigger(() -> xboxDriveController.getPOV() == 90)
        .onTrue(new PanLeftRightToReefTargetRobotCentric(-0.25))
        .onFalse(new StopRobot());

    new Trigger(() -> xboxDriveController.getPOV() == 270)
        .onTrue(new PanLeftRightToReefTargetRobotCentric(0.25))
        .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new InstantCommand(driveSubsystem::setRobotCentricTrue))
      .onFalse(new InstantCommand(driveSubsystem::setRobotCentricFalse));
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
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("Blu-BargeToReef11"))
      .onFalse(new StopRobot());
  }

  // public void testAutoChoate() throws Exception {

  //   System.out.println("Def CH");
  //   new JoystickButton(driveStick1, 12)
  //     .onTrue(new AutoStraightTrajectoryToReef8())
  //     .onFalse(new StopRobot());
  //     System.out.println("End Def CH");
  // }

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


    // new JoystickButton(driveStick1, 7)
    //   .onTrue(
    //     new ArmToPositionAndHold(ArmPositions.CoralCruise)
    //     .andThen(new WaitCommand(1))
    //     .andThen(new ArmToPositionAndHold(ArmPositions.ReefLevelFour))
    //     .andThen(new WaitCommand(1))
    //     .andThen(new ArmToPositionAndHold(ArmPositions.CoralCruise)) 
    //   )
    //   .onFalse(new StopArm());

  }

  public void testClimber() throws Exception {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new ClimberStartWithSpeed(0.2))
      .onFalse(new StopClimber());
  }


  public void calibrateElevator() {
    // new JoystickButton(driveStick1, 1)
    // .onTrue(new CalibrateElevatorDeterminekG())
    // .onFalse(new StopElevatorAndHold());

    // new JoystickButton(driveStick1, 2)
    // .onTrue(new StopElevator());
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
    // new JoystickButton(driveStick1, 1)
    //   .onTrue(new RunTrajectorySequenceRobotAtStartPoint("Blu-BargeToReef8"))
    //   .onFalse(new StopRobot());
  }

  public void testMohawk() throws Exception { //also some trajectory testing
    // new JoystickButton(driveStick1, 12)
    //    .onTrue(new AlgaePickupReadyFromHigh());

    // new JoystickButton(driveStick1, 11)
    //    .onTrue(new AlgaePickupReadyFromLow());

    // new JoystickButton(driveStick1, 10)
    //    .onTrue(new AlgaeSpitOut())
    //    .onFalse(new StopArm());
    
    // new JoystickButton(xboxDriveController, 5)
    //    .onTrue(new IntakeCoralAndMoveToCruisePositionSequence());

    // new JoystickButton(driveStick1, 8)
    //    .onTrue(new CoralPlaceOnTwo())
    //    .onFalse(new StopArm());

    // new JoystickButton(driveStick1, 7)
    //    .onTrue(new CoralPlaceOnThree())
    //    .onFalse(new StopArm());

    // new JoystickButton(driveStick1, 6)
    //    .onTrue(new CoralPlaceOnFour());

    // new JoystickButton(driveStick1, 5)
    //    .onTrue(new TeleopPanReefLeft())
    //    .onFalse(new StopArm());

    // new JoystickButton(driveStick1, 6)
    //    .onTrue(new TeleopPanReefRight())
    //    .onFalse(new StopArm());
    
    // new JoystickButton(driveStick1, 4)
    //    .onTrue(new StopArm());
    
    // new JoystickButton(driveStick1, 3)
    //    .onTrue(new CoralIntakeReadySequence());

    // new JoystickButton(driveStick1,11 )
    //   .onTrue(new AlgaeToBarge());

    // new JoystickButton(driveStick1, 7)
    //   .onTrue(new ArmToPositionAndHold(ArmConstants.ArmPositions.CoralCruise)); 

    // new JoystickButton(driveStick1, 10)
    //   .onTrue(new ArmToPositionAndHold(ArmConstants.ArmPositions.BargeEnd).alongWith(
    //     new IntakeAlgaeRollOutBargeCommand())
    //     .andThen(new WaitCommand(0.3))
    //     .andThen(new StopIntake()));

    new JoystickButton(driveStick1, 8)
      .onTrue(new InstantCommand(()->driveSubsystem.setOdometryPoseToSpecificPose(
          new Pose2d(10.654, 1.910, Rotation2d.fromDegrees(0)))));
    // new JoystickButton(driveStick1,12 )
    //   .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward-90Turn"))
    //   .onFalse(new StopRobot());
    
    // new JoystickButton(driveStick1,12 )
    //   .onTrue(new AutonomousTrajectory2Poses(new Pose2d(1, 5, Rotation2d.fromDegrees(0)), new Pose2d(2, 5, Rotation2d.fromDegrees(30))))      
    //   .onFalse(new StopRobot());

    // new JoystickButton(driveStick1, 3)
    //   .onTrue(new StopElevator().alongWith(new StopIntake()));

    // new JoystickButton(driveStick1, 4)
    //   .onTrue(new IntakeShootCommand());

    // new JoystickButton(driveStick1,12 )
    //    .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward-90Turn"))
    //    .onFalse(new StopRobot());

    // new JoystickButton(driveStick1, 11)
    //   .onTrue(new IntakeCoralOutCommand(IntakeConstants.coralShootingPowerL23)); 

    // new JoystickButton(driveStick1, 1)
    //   .onTrue(new ClimberStartWithSpeed(ClimberConstants.climbUpPower));

    // new JoystickButton(driveStick1, 12)
    //   .onTrue(new ClimberStartWithSpeed(0.2))
    //   .onFalse(new StopClimber());

  }

  // public void testChoate(){
  //   new JoystickButton(driveStick1, 10)
  //     .onTrue(new CoralPlaceOnFour())
  //     .onFalse(new StopArm());
  // }


  public void calibrateChassisDeadband() {
    // new JoystickButton(driveStick1, 1)
    //   .onTrue(new CalibrateChassisLinearDeadband())
    //   .onFalse(new StopRobot());
    // new JoystickButton(driveStick1, 1)
    //    .onTrue(new CalibrateChassisAngularDeadband())
    //    .onFalse(new StopRobot());
  }

  public void testVisionCoordoinates() {
    System.out.println("****Poses:  ");
    // System.out.println(llVisionSubsystem.getKnownPose("RobotBluReef1Left"));
    // System.out.println(llVisionSubsystem.getKnownPose("RobotBluReef1Right"));

    List<String> keys = new ArrayList<>();
    for (String k : RobotPoseConstants.visionRobotPoses.keySet()) {
      keys.add(k);
    }
    // for (String key : keys) {
    //   System.out.println(key + RobotPoseConstants.visionRobotPoses.get(key));
    // }
  }

  public void tryPPTestCalibration() {
    new JoystickButton(driveStick1, 12)
      .onTrue(runTrajectoryPathPlannerWithForceResetOfStartingPose("Red-BargetoReef11", true,false))
    //     .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef11ToCoralTop",false))
    //     .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralTopToReef10", false)))
      .onFalse(new StopRobot()); 
  }

  public void tryPPTest() {
    new JoystickButton(driveStick1, 12)
       .onTrue(
       // start with setting odometry to pose 
       new InstantCommand( ()-> driveSubsystem.setOdometryToIdealPoseFromTrajectory("Blu-BargeToReef11"))
        // just in case - wait 0.1s for the pose to take place
        .andThen(new WaitCommand(0.1))
        .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-BargeToReef11", true,false))
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopMoveToL4RotateArm()) // try placing Coral on L4
        .andThen(new TeleopEjectCoralBringArmToCruiseElevatorDown())//TODO: needs to be looked over, especially if we have to 
                                                                    //add the trajectory which will make the bot go backwards 
                                                                    //before putting the elevator down. 
        .andThen(new WaitCommand(0.1))
        .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef11ToCoralTop",false,false))
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopCoralIntakeSequence())
        .andThen(new WaitCommand(0.1))
        .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralTopToReef10", false,false))
        .andThen(new TeleopMoveToL4RotateArm())  // try placing Coral on L4
        .andThen(new TeleopEjectCoralBringArmToCruiseElevatorDown())
       )
       .onFalse(new StopRobot());
  }

  public void tryPPTest2() {
    new JoystickButton(driveStick1, 12)
       .onTrue(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-BargeToReef11", true, false)
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopMoveToL4RotateArm())
        .andThen(new TeleopEjectCoralBringArmToCruiseElevatorDown())//TODO: needs to be looked over, especially if we have to
                                                                    //add the trajectory which will make the bot go backwards
                                                                    //before putting the elevator down.
        .andThen(new WaitCommand(0.1))
        .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef11ToCoralTop",false, false))
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopCoralIntakeSequence())
        .andThen(new WaitCommand(0.1))
        .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralTopToReef10", false, false))
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopMoveToL4RotateArm())
        .andThen(new TeleopEjectCoralBringArmToCruiseElevatorDown())
        .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef10ToCoralTop", false, false))
        .andThen(new TeleopCoralIntakeSequence())
        .andThen(runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralTopToReef9", false, false))
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopMoveToL4RotateArm())
        .andThen(new TeleopEjectCoralBringArmToCruiseElevatorDown()))
       .onFalse(new StopRobot());
  }


  public static Command runTrajectoryPathPlannerWithForceResetOfStartingPose(String tr,
      boolean shouldResetOdometryToStartingPose, boolean flipTrajectory) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(tr);

      if (flipTrajectory) {
        path = path.flipPath();
      }

      Pose2d startPose = path.getStartingHolonomicPose().get();
      driveSubsystem.setOdometryPoseToSpecificPose(startPose); // reset odometry, as PP may not do so

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      if (! shouldResetOdometryToStartingPose) {
        return AutoBuilder.followPath(path);
      } else { // reset odometry the right way
        return Commands.sequence(AutoBuilder.resetOdom(startPose), AutoBuilder.followPath(path));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  /**
   * Run trajectory between 2 poses in native PathPlanner way
   * @param startPose
   * @param endPose
   * @param shouldResetOdometryToStartingPose
   * @return
   */
  public static Command runTrajectory2Poses(Pose2d startPose, Pose2d endPose,
      boolean shouldResetOdometryToStartingPose) {
    try {
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          AutoConstants.pathCconstraints,
          new IdealStartingState(0, startPose.getRotation()),
          new GoalEndState(0, endPose.getRotation()));
      path.preventFlipping = true;
      driveSubsystem.setOdometryPoseToSpecificPose(startPose); // reset odometry, as PP may not do so
      if (!shouldResetOdometryToStartingPose) {
        return AutoBuilder.followPath(path);
      } else { // reset odometry the right way
        System.out.println("== Driving from "+startPose+" to "+endPose);
        return Commands.sequence(AutoBuilder.resetOdom(startPose), AutoBuilder.followPath(path));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

/**
 * Run trajectory from current location as determined by cameras to known location, such as known AT placement
 * @param knownLocation
 * @param assumeLocation - if the pose cannot be determined by cameras, assume that current odometry location is correct
 * @return
 */
  public static Command runTrajCurLocToKnownLoc(Pose2d knownLocation, boolean assumeLocation) {
    try {
      if (RobotContainer.llVisionSubsystem.isAprilTagVisibleBySomeCamera()) {
        return (
          runTrajectory2Poses(
            RobotContainer.llVisionSubsystem.getBestPoseAllCameras(),
            knownLocation,
            true // reset odometry, since the pose determination is dynamic
          )
        );
      } else {
        if (assumeLocation) {
          return 
            runTrajectory2Poses(
              driveSubsystem.getPose(),
              knownLocation,
              true // reset odometry, since the pose determination is dynamic
            );
        } else {
          return new PrintCommand("Current position cannot be determined by cameras");
        }
      }
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
