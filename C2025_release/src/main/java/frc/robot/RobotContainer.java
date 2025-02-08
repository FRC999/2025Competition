// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmRunWithSpeed;
import frc.robot.commands.ArmToPositionAndHold;
import frc.robot.commands.AutonomousTrajectory2Poses;
import frc.robot.commands.CalibrateArmMoveManually;
import frc.robot.commands.CalibrateDriveTrainTurnMinimumPower;
import frc.robot.commands.CalibrateElevatorDeterminekG;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ElevatorRunWithSpeed;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.StartClimberWithSpeed;
import frc.robot.commands.StopArm;
import frc.robot.commands.StopClimber;
import frc.robot.commands.StopElevator;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TurnToRelativeAngleTrapezoidProfile;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  public static Controller xboxDriveController;
  public static Controller xboxGPMController;
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
  }

  private void configureDriverInterface(){
    driveStick1 = new Joystick(0); //TODO: ONLY FOR TESTING; MUST BE COMMENTED FOR COMP
    xboxDriveController = new Controller(ControllerDevice.XBOX_CONTROLLER);
    xboxGPMController = new Controller(ControllerDevice.XBOX_CONTROLLER_GPM);
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
    //return -xboxController.getLeftStickY();
    return -xboxDriveController.getRightStickY();
  }

  private double getDriverYAxis() {
    //return -xboxController.getLeftStickX();
    return -xboxDriveController.getRightStickX();
  }

  private double getDriverOmegaAxis() {
    //return -xboxController.getLeftStickOmega();
    return -xboxDriveController.getLeftStickX();
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
      testAuto();
    }
    catch (Exception e) {
       System.out.println("test auto error: " + e);
    }

    testTurn();
    setYaws();
  }

  public void setYaws() {
    new JoystickButton(xboxDriveController, 8)
      .onTrue(new InstantCommand(() -> driveSubsystem.zeroYaw()));
  }

   public void testAuto() throws Exception {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeter90"))
      .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 2)
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward"))
      .onFalse(new StopRobot());
  }

  public void testArm() throws Exception {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new ArmRunWithSpeed(0.2))
      .onFalse(new StopArm());
  }

  public void testElevator() throws Exception {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new ElevatorRunWithSpeed(0.2))
      .onFalse(new StopElevator());
  }

  public void testClimber() throws Exception {
    new JoystickButton(xboxDriveController, 1)
      .onTrue(new StartClimberWithSpeed(0.2))
      .onFalse(new StopClimber());
  }

  public void calibrateElevator() {
    new JoystickButton(xboxDriveController, 1)
    .onTrue(new CalibrateElevatorDeterminekG())
    .onFalse(new StopElevator());
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return null; 
  }
}
