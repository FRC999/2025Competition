// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.Constants.SwerveConstants.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants.SwerveChassis.SwerveModuleConstantsEnum;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX,TalonFX,CANcoder> implements Subsystem {


  public static InterpolatingDoubleTreeMap chassisAngularVelocityConversion = new InterpolatingDoubleTreeMap(); 
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = true; // red/blue side boolean decider
  private boolean isRobotCentric = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveChassis.MaxSpeed * SwerveChassis.chassisLinearMoveDeadband).withRotationalDeadband(SwerveChassis.MaxAngularRate * SwerveChassis.chassisAngularMoveDeadband) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
   private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(SwerveChassis.MaxSpeed * SwerveChassis.chassisLinearMoveDeadband).withRotationalDeadband(SwerveChassis.MaxAngularRate * SwerveChassis.chassisAngularMoveDeadband) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  Pigeon2 imu;
  private double trajectoryAdjustmentIMU; // This is the value we need to adjust the IMU by after Trajectory
  // is completed

  private double previousOmegaRotationCommand;

  

  private final SysIdRoutine mSysIdRoutine = 
    new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.of(0.1).per(Second),
           Volts.of(0.1),
           Seconds.of(50)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Voltage volts) -> {
                        this.turnToAngleWithVolt(volts.in(Volts));
                },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("chassis-angleMotor")
                    .voltage(Volts.of(previousOmegaRotationCommand))
                    .angularPosition(Radians.of(this.getAngularPosition()))
                    .angularVelocity(RadiansPerSecond.of(this.getAngularVelocity()));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(double OdometryUpdateFrequency) {

    
        super(
          TalonFX::new, TalonFX::new, CANcoder::new,
          TunerConstants.DrivetrainConstants, OdometryUpdateFrequency, configureSwerveChassis());
        imu = this.getPigeon2();
    }

  public DriveSubsystem() {

        super(
          TalonFX::new, TalonFX::new, CANcoder::new,
          TunerConstants.DrivetrainConstants, (SwerveModuleConstants[]) configureSwerveChassis());

        if (!EnabledSubsystems.chasis) {
          return;
        }

        imu = this.getPigeon2();
        this.registerTelemetry(this::telemeterize);

        setChassisAngularVelocityConversion();

        configureAutoBuilder();
    }

   /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return mSysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return mSysIdRoutine.dynamic(direction);
  }

  @SuppressWarnings("unchecked")
  public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] configureSwerveChassis() {
    return new SwerveModuleConstants[]{
      
      // Front Left  
      TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD0.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD0.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD0.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD0.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD0.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD0.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD0.isCANCoderIverted()
            ),
            
        // Front Right
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD1.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD1.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD1.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD1.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(-SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD1.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD1.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD1.isCANCoderIverted()
            ),
            
        // Back Left
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD2.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD2.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD2.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD2.getAngleOffset()),
            Meters.of(-SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD2.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD2.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD2.isCANCoderIverted()
            ),
            
        // Back Right
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD3.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD3.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD3.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD3.getAngleOffset()),
            Meters.of(-SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(-SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD3.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD3.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD3.isCANCoderIverted()
            )
    };
  }

  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s) {
    //System.out.println("X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s/SwerveChassis.MaxAngularRate);
    //SmartDashboard.putString("Manual Drive Command Velocities","X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s);
    this.setControl(
      drive.withVelocityX(xVelocity_m_per_s)
        .withVelocityY(yVelocity_m_per_s)
        .withRotationalRate(omega_rad_per_s)
    );
    previousOmegaRotationCommand = omega_rad_per_s / SwerveChassis.MaxAngularRate;
  }

  public void driveRobotCentric(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s) {
    //System.out.println("X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s/SwerveChassis.MaxAngularRate);
    //SmartDashboard.putString("Manual Drive Command Velocities","X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s);
    this.setControl(
      driveRobotCentric.withVelocityX(xVelocity_m_per_s)
        .withVelocityY(yVelocity_m_per_s)
        .withRotationalRate(omega_rad_per_s)
    );
    previousOmegaRotationCommand = omega_rad_per_s / SwerveChassis.MaxAngularRate;
  }

    /** 
   * Field Centric Pose of the chassis
   * We get it from odometry, rather than sensors. That means commands that use it must ensure that
   * odometry was properly updated.
  */
  public Pose2d getPose() {
    //System.out.println("cp: " + this.getState().Pose);
    return this.getState().Pose;
  }

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds ctrSpeed =  this.getState().Speeds;
    //ctrSpeed.omegaRadiansPerSecond = getChassisAngularVelocityConversion(ctrSpeed.omegaRadiansPerSecond);
    return ctrSpeed;
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    drive(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond
    );
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds, DriveFeedforwards driveFeedforwards) {
    //SmartDashboard.putString("ChassisACommand", speeds.toString());
    System.out.println("AC: " + speeds.toString());
    System.out.println("Y: " + getYaw());
    drive(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      //getChassisAngularVelocityConversion(speeds.omegaRadiansPerSecond)
      speeds.omegaRadiansPerSecond
    );
  }

  public TalonFX getDriveMotor(int modNum){
    return this.getModule(modNum).getDriveMotor();
  }

  public TalonFX getTurnMotor(int modNum){
    return this.getModule(modNum).getSteerMotor();
  }

  public double getDriveEncoder(int modNum){
    return this.getModule(modNum).getDriveMotor().getRotorPosition().getValueAsDouble();
  }

  public double getDriveEncoderSI(int modNum){
    return this.getModule(modNum).getDriveMotor().getRotorPosition().getValueAsDouble()
      *SwerveChassis.metersPerRotationFX;
  }

  public double getTurnEncoder(int modNum){
    return this.getModule(modNum).getSteerMotor().getRotorPosition().getValueAsDouble();
  }

  public double getDriveVelocity(int modNum){
    return this.getModule(modNum).getDriveMotor().getRotorVelocity().getValueAsDouble();
  }

 public double getDriveVelocitySI(int modNum){
    return this.getModule(modNum).getDriveMotor().getRotorVelocity().getValueAsDouble()
      *SwerveChassis.metersPerRotationFX;
  } 

  public double getTurnVelocity(int modNum){
    return this.getModule(modNum).getSteerMotor().getRotorVelocity().getValueAsDouble();
  }

  public double getCancoderAbsolute(int modNum){
    return this.getModule(modNum).getEncoder().getAbsolutePosition().getValueAsDouble();
  }

  public double getCancoderRelative(int modNum){
    return this.getModule(modNum).getEncoder().getPosition().getValueAsDouble();
  }

   public double getCancoderAbsoluteSI(int modNum){
    return this.getModule(modNum).getEncoder().getAbsolutePosition().getValueAsDouble()*360.0;
  }

  public double getCancoderRelativeSI(int modNum){
    return ( this.getModule(modNum).getEncoder().getPosition().getValueAsDouble()
            - getModuleEnum(modNum).getAngleOffset())*360.0;
  }

  public SwerveModuleConstantsEnum getModuleEnum(int modNum){
    switch(modNum){
      case 0: 
        return SwerveModuleConstantsEnum.MOD0;
      case 1: 
        return SwerveModuleConstantsEnum.MOD1;
      case 2:
        return SwerveModuleConstantsEnum.MOD2;
      default:
        return SwerveModuleConstantsEnum.MOD3;
    }
  }

  public void stopRobot(){
    drive(0,0,0);
  }

  public double getOmegaRoationCommand() {
    return previousOmegaRotationCommand;
  }
  /**
   * Note that all IMU methods that take or return values should do so in SI
   * units.
   */

  public double getPitch() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // return ypr[1];

    // Front UP - positive Pitch
    return -imu.getPitch().getValueAsDouble();
  }

  public static double getChassisAngularVelocityConversion(double velocity){
    return chassisAngularVelocityConversion.get(velocity);
  }

  public static void setChassisAngularVelocityConversion() {
    chassisAngularVelocityConversion.put(0.43, 0.2*Math.PI);
    chassisAngularVelocityConversion.put(0.60, 0.25*Math.PI);
    chassisAngularVelocityConversion.put(0.78, 0.3*Math.PI);
    chassisAngularVelocityConversion.put(0.96, 0.35*Math.PI);
    chassisAngularVelocityConversion.put(1.17, 0.4*Math.PI);
    chassisAngularVelocityConversion.put(1.39, 0.45*Math.PI);
    chassisAngularVelocityConversion.put(1.57, 0.5*Math.PI);
    chassisAngularVelocityConversion.put(1.75, 0.55*Math.PI);
    chassisAngularVelocityConversion.put(1.92, 0.6*Math.PI);
    chassisAngularVelocityConversion.put(2.10, 0.65*Math.PI);
    chassisAngularVelocityConversion.put(2.26, 0.7*Math.PI);
    chassisAngularVelocityConversion.put(2.43, 0.75*Math.PI);
    chassisAngularVelocityConversion.put(2.60, 0.8*Math.PI);
    chassisAngularVelocityConversion.put(3.28, 1.0*Math.PI);
    chassisAngularVelocityConversion.put(3.94, 1.2*Math.PI);
    chassisAngularVelocityConversion.put(4.61, 1.4*Math.PI);
    chassisAngularVelocityConversion.put(4.94, 1.5*Math.PI);
    chassisAngularVelocityConversion.put(5.27, 1.6*Math.PI);
    chassisAngularVelocityConversion.put(5.96, 1.8*Math.PI);
    chassisAngularVelocityConversion.put(6.63, 2.0*Math.PI);
  }

  /**
   * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
   * axis that goes straight forward)
   * 
   * @return
   */
  public double getRoll() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // return ypr[2];

    // Left UP - positive Roll
    return imu.getRoll().getValueAsDouble();
  }

  /**
   * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
   * robot is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getYaw() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // System.out.println(ypr[0]);
    // return ypr[0];

    return imu.getYaw().getValueAsDouble(); // With Pigeon2 this method returns values in degrees
  }

  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  /**
   * Zeroes the yaw of the robot
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double previousYaw = getYaw();
    System.out.println("Old Yaw: " + previousYaw);
    if (RobotContainer.isAllianceRed 
        && RobotContainer.isReversingControllerAndIMUForRed
      ) {
      System.out.println("Yaw 180 " + RobotContainer.isAllianceRed);

      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = imu.setYaw(180.0);
        if (status.isOK())
          break;
      }
      if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }

      // Reset IMU pose; may need to remove for the competition
      setCurrentOdometryPoseToSpecificRotation(180);
  
    } else 
    {
      System.out.println("Yaw NOT 180 " + RobotContainer.isAllianceRed);

      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = imu.setYaw(0);
        if (status.isOK())
          break;
      }
      if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }

      setCurrentOdometryPoseToSpecificRotation(0);
    }
    System.out.println("New Yaw: " + imu.getYaw());
    return previousYaw;
  }

  public double setYaw(double y) {
    double previousYaw = getYaw();
    StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = imu.setYaw(y);
        if (status.isOK())
          break;
      }
      if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }
    return previousYaw;
  }

  public Rotation2d getRotation2d() {
    return imu.getRotation2d();
  }

  public double getPigeon2Speed() {
    return Math.toRadians(imu.getAngularVelocityZWorld().getValueAsDouble());
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // if (RobotContainer.isAllianceRed && RobotContainer.isReversingControllerAndIMUForRed) {
    //   imu.setYaw(180.0);
    // } else 
    {
      imu.setYaw(0);
    }
    System.out.println("Yaw and Fused Heading set");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180 in Rotation2d format
   */
  public Rotation2d getHeading() {
    return imu.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return imu.getAngularVelocityZWorld().getValueAsDouble(); // getAngularVelocityZWorld - CCW+
  }

  /**
   * This method is used when we want to "snap" the chassis to a trajectory start,
   * meaning
   * assuming that the robot is at the starting point of the trajectory.
   * Here we remember starting Yaw before trajectory so it can be restored
   * back after trajectory
   * 
   * @param y - starting Yaw of the trajectory
   * @return - old value of the Yaw (we do not currently use it)
   */
  public double setYawForTrajectory(double y) {
    trajectoryAdjustmentIMU = this.getPose().getRotation().getDegrees() - y;

    // alex test
    System.out.println("---- Trajectory AdjustmentIMU: "+ trajectoryAdjustmentIMU
      + " CP: "+ this.getPose().getRotation().getDegrees() + " A: " + y) ;
    
    setYaw(y);
    this.resetRotation(Rotation2d.fromDegrees(y));


    return y; // our own setYaw that returns old angle
  }

  /**
   * Once the trajectory is done, we want to readjust the Yaw considering the
   * value that we "remember", so
   * the field-centric drive axis will not change. That may allow one to drive
   * automated trajectories in teleop
   * without losing the Yaw direction.
   */
  public void restoreYawAfterTrajectory() {
    double currentYaw = this.getPose().getRotation().getDegrees();
    setYaw(currentYaw + trajectoryAdjustmentIMU);
    System.out.println("Final pose: " + this.getPose());
    System.out.println(
        "Restoring original IMU after trajectory " + (this.getPose().getRotation().getDegrees() + trajectoryAdjustmentIMU));
    this.resetPose(
      new Pose2d(0, 0, 
        Rotation2d.fromDegrees(this.getPose().getRotation().getDegrees() + trajectoryAdjustmentIMU))
        );
    this.resetRotation(Rotation2d.fromDegrees(currentYaw + trajectoryAdjustmentIMU));
  }

  /**
   * Set odometry to a specified field-centric Pose2d
   * You may need to do so for the trajectory driving, if you want the robot to assume being at the
   * start of the trajectory.
   * Be aware that on-going odometry updates use IMU. So, your odometry yaw may change incorrectly
   * later if the current yaw is not reset properly on the IMU first.
   * 
   * Note that the pose is from Blue Alliance perspective
   */
  public void resetOdometry(Pose2d pose) {
    this.resetPose(pose);
  }

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    SmartDashboard.putString("P:", state.Pose.toString());
    // SmartDashboard.putString("M0A:", state.ModuleStates[0].toString());
    // SmartDashboard.putString("M1A:", state.ModuleStates[1].toString());

  }

  public void turnToAngleWithVolt(double power) {
    drive(0, 0, power*SwerveChassis.MaxAngularRate);
  }

  public double getAngularPosition() {
    return this.getState().Pose.getRotation().getRadians();
  }

  public double getAngularVelocity() {
    return this.getState().Speeds.omegaRadiansPerSecond;
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) -> setControl(
              m_pathApplyRobotSpeeds.withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the
          // case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
      );
    } catch (Exception ex) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public void setRobotCentricTrue() {
    isRobotCentric = true;
  }

  public void setRobotCentricFalse() {
    isRobotCentric = false;
  }

  public boolean getRobotCentric() {
    return isRobotCentric;
  }

  public void setOdometryPoseToSpecificPose(Pose2d p) {
    this.resetPose(p);
  }

  /**
   * Take trajectory, extract starting pose, reset odometry to it
   * 
   * @param tr
   */
  public void setOdometryToIdealPoseFromTrajectory(String tr) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(tr);

      Pose2d startPose = path.getStartingHolonomicPose().get();
      setOdometryPoseToSpecificPose(startPose); // reset odometry, as PP may not do so
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  public void setCurrentOdometryPoseToSpecificRotation(double degrees) {
    Pose2d currentPose = this.getPose();
    Pose2d newPose = new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromDegrees(degrees));
    setOdometryPoseToSpecificPose(newPose);
  }

  public Pose2d getVisionAidedOdometryPose() {
    if (RobotContainer.llVisionSubsystem.isAprilTagVisibleBySomeCamera()) {
      return RobotContainer.llVisionSubsystem.getBestPoseAllCameras();
    } else {
      return getPose();
    }
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        // if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        //     DriverStation.getAlliance().ifPresent((allianceColor) -> {
        //         this.setOperatorPerspectiveForward(
        //                 allianceColor == Alliance.Red 
        //                         ? SwerveChassis.redAlliancePerspectiveRotation
        //                         : SwerveChassis.blueAlliancePerspectiveRotation);
        //         hasAppliedOperatorPerspective = true;
        //     });
        // }

        //System.out.println("CS1: " + getChassisSpeeds());
  }
}
