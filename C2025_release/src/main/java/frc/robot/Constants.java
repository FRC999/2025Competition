// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static final class EnabledSubsystems {
		public static final boolean arm = true;
		public static final boolean climber = false;
		public static final boolean elevator = false;
		public static final boolean intake = true;
		public static final boolean chasis = true;
		public static final boolean reef = false;
		public static final boolean ll = false;
	}

	public static final class DebugTelemetrySubsystems {
		
		public static final boolean odometry = true;
		public static final boolean imu = true;
		public static final boolean arm = true;
		public static final boolean intake = true;
		public static final boolean elevator = false;
		public static final boolean climber = false;
		public static final boolean chasis = true;
		public static final boolean reef = false;
		public static final boolean ll = false;
	}


	public static class SwerveConstants {
		public static class TunerConstants {
			public static final double steerGainsKP = 100;
			public static final double steerGainsKI = 0;
			//public static final double steerGainsKD = 2.0;
			public static final double steerGainsKD = 0.5;
			//public static final double steerGainsKS = 0.2;
			public static final double steerGainsKS = 0.1;
			//public static final double steerGainsKV = 2.66;
			public static final double steerGainsKV = 1.59;
			public static final double steerGainsKA = 0;

			private static final Slot0Configs steerGains = new Slot0Configs()
					.withKP(steerGainsKP).withKI(steerGainsKI).withKD(steerGainsKD) //TODO: generated code has KD 0.5
					.withKS(steerGainsKS).withKV(steerGainsKV).withKA(steerGainsKA) //TODO: generated code has KS 0.1, KV as 1.59
					.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

			public static final double driveGainsKP = 0.1;
			public static final double driveGainsKI = 0;
			public static final double driveGainsKD = 0;
			public static final double driveGainsKS = 0;
			public static final double driveGainsKV = 0.124;
			//public static final double driveGainsKA = 0;

			private static final Slot0Configs driveGains = new Slot0Configs()
					.withKP(driveGainsKP).withKI(driveGainsKI).withKD(driveGainsKD)
					.withKS(driveGainsKS).withKV(driveGainsKV);

			private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
			private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
			
			private static final DriveMotorArrangement driveMotorType = DriveMotorArrangement.TalonFX_Integrated;
			private static final SteerMotorArrangement steerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    		// The remote sensor feedback type to use for the steer motors;
    		// When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    		private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

			private static final Current slipCurrent = Amps.of(120.0);

			private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
			private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									// Swerve azimuth does not require much torque output, so we can set a
									// relatively low
									// stator current limit to help avoid brownouts without impacting performance.
									.withStatorCurrentLimit(Amps.of(60))
									.withStatorCurrentLimitEnable(true));

			private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

			// Theoretical free speed (m/s) at 12v applied output;
			// This needs to be tuned to your individual robot
			//public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(6.21); //TODO: the old value was 5.21
			public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(5.21); //TODO: the old value was 5.21

			// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
			// This may need to be tuned to your individual robot
			private static final double kCoupleRatio = 3; //TODO: the old value was 3.5714285714285716
			//private static final double kCoupleRatio = 3.5714285714285716; //TODO: the old value was 3.5714285714285716

			private static final double kDriveGearRatio = 5.142857142857142*(0.13/5.22); //TODO" the old value was 6.122448979591837 * (1/2.09)
			//private static final double kDriveGearRatio = 6.122448979591837;
			private static final double kSteerGearRatio = 12.8 ; //TODO: the old value was 21.428571428571427
			private static final Distance wheelRadius = Inches.of(SwerveChassis.WHEEL_DIAMETER/2.0); //TODO: the old value was Inches.of(5.33 / 5.71)

			private static final boolean kInvertLeftSide = false;
			private static final boolean kInvertRightSide = true;

			public static final CANBus kCANBus = new CANBus("canivore1", "./logs/example.hoot");

			// These are only used for simulation
			private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
			private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
			// Simulated voltage necessary to overcome friction
			private static final Voltage kSteerFrictionVoltage = Volts.of(0.2); //TODO: old value was 0.25
			private static final Voltage kDriveFrictionVoltage = Volts.of(0.2); //TODO: old value was a 0.25

			public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
					.withCANBusName(kCANBus.getName())
					.withPigeon2Id(IMUConstants.kPigeonId) 
					.withPigeon2Configs(IMUConstants.pigeonConfigs);

			public static final SwerveModuleConstantsFactory<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> ConstantCreator = 
				new SwerveModuleConstantsFactory<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> ()
					.withDriveMotorGearRatio(kDriveGearRatio)
					.withSteerMotorGearRatio(kSteerGearRatio)
					.withCouplingGearRatio(kCoupleRatio)
					.withWheelRadius(wheelRadius)
					.withSteerMotorGains(steerGains)
					.withDriveMotorGains(driveGains)
					.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
					.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
					.withSlipCurrent(slipCurrent)
					.withSpeedAt12Volts(speedAt12Volts)
					.withDriveMotorType(driveMotorType)
					.withSteerMotorType(steerMotorType)
					.withFeedbackSource(steerFeedbackType)
					.withDriveMotorInitialConfigs(driveInitialConfigs)
					.withSteerMotorInitialConfigs(steerInitialConfigs)
					.withEncoderInitialConfigs(cancoderInitialConfigs)
					.withSteerInertia(kSteerInertia)
					.withDriveInertia(kDriveInertia)
					.withSteerFrictionVoltage(kSteerFrictionVoltage)
					.withDriveFrictionVoltage(kDriveFrictionVoltage)
					;

		}

		public static class SwerveChassis {

			public static final double TRACK_WIDTH = Meters.convertFrom(18.00, Inches); // left to right
			public static final double WHEEL_BASE = Meters.convertFrom(18.00, Inches); // front to back
			public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
			public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

			public static final double MaxSpeed = TunerConstants.speedAt12Volts.magnitude(); // kSpeedAt12VoltsMps
																								// desired top speed
			public static final double maxAcceleration = 41.68; // this is Max linear acceleration units: m/s^2
			public static final double MaxAngularRate = 2.0 * Math.PI; // 3/4 of a rotation per second max angular
																		// velocity
			public static final double maxAngularAcceleration = 37.6992; // this is max angular acceleration units:
																		// rad/s^2
			public static final double robotMass = 56.7; // kg
			public static final double robotInertia = 60.0; // KG*M^2 - for rotation
			public static final double wheelCOF = 1.2; // coefficient of friction for the wheels; colsons on carpet is
														// 1.0
			public static final double chassisLinearMoveDeadband = 0.02; //determined by calibration method 
			public static final double chassisAngularMoveDeadband = 0.05; //determined by calibration method
			// Customize the following values to your prototype
			public static final double metersPerRotationFX = (5.589/89.11199955)*(5.589/5.716); // measure this number on the robot - remeasure on carpet
			// drive motor only
			public static final double degreePerRotationFX = (1.0 / 122.11575) * 2048; // Angle motor only
			// On our swerve prototype 1 angular rotation of
			// the wheel = 1 full rotation of the encoder

			/**
			 * Drive Motor PID. Assumed to be the same for all drive motors
			 * These PID constants are only used for auto trajectory driving, and not
			 * teleop.
			 * We found that changing them a bit will not have a substantial impact on the
			 * trajectory with PathPlanner
			 * even if a trajectory includes a holonomic component.
			 */
			//public static final double DRIVE_CHASSIS_KP = 3.5;
			public static final double DRIVE_CHASSIS_KP = 5.0;
			public static final double DRIVE_CHASSIS_KI = 0.00;
			public static final double DRIVE_CHASSIS_KD = 0;

			/**
			 * Angle Motor PID. Assumed to be the same for all angle motors
			 * These PID constants are only used for auto trajectory driving, and not
			 * teleop.
			 * Changes to these constants will have a substantial impact on the precision of
			 * your
			 * trajectory if it includes holonomic rotation.
			 * Make sure to test the values and adjust them as needed for your robot.
			 */
			public static final double ANGLE_CHASSIS_KP = 6.25;
			public static final double ANGLE_CHASSIS_KI = 0.4;
			public static final double ANGLE_CHASSIS_KD = 0.7;

			public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
					.withDeadband(MaxSpeed * chassisLinearMoveDeadband).withRotationalDeadband(MaxAngularRate * chassisAngularMoveDeadband) // Add a 10% deadband
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
			

			/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
			public static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
			/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
			public static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

			public static enum SwerveModuleConstantsEnum {
				MOD0( // Front Left,
						3, // driveMotorID
						4, // angleMotorID
						31, // CanCoder Id
						// -0.296142578125, // angleOffset of cancoder to mark zero-position
						0.022582890625, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				),
				MOD1( // Front Right
						1, // driveMotorID
						2, // angleMotorID
						30, // CanCoder Id
						// 0.041015625, // angleOffset of cancoder to mark zero-position
						-0.3797604921875, // angleOffset of cancoder to mark zero-position
						true, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				),
				MOD2( // Back Left
						7, // driveMotorID
						8, // angleMotorID
						33, // CanCoder Id
						// -0.296142578125, // angleOffset of cancoder to mark zero-position
						0.421386796875, // angleOffset of cancoder to mark zero-position
						false, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				),
				MOD3( // Back Right
						5, // driveMotorID
						6, // angleMotorID
						32, // CanCoder Id
						// 0.326171875, // angleOffset of cancoder to mark zero-position
						//0.0576171875, // angleOffset of cancoder to mark zero-position
						0.088256890625, // angleOffset of cancoder to mark zero-position
						true, // Inversion for drive motor
						false, // Inversion for angle motor
						false // inversion for CANcoder
				);

				private int driveMotorID;
				private int angleMotorID;
				private int cancoderID;
				private double angleOffset;
				private boolean driveMotorInverted;
				private boolean angleMotorInverted;
				private boolean cancoderInverted;

				SwerveModuleConstantsEnum(int d, int a, int c, double o,
						boolean di, boolean ai, boolean ci) {
					this.driveMotorID = d;
					this.angleMotorID = a;
					this.cancoderID = c;
					this.angleOffset = o;
					this.driveMotorInverted = di;
					this.angleMotorInverted = ai;
					this.cancoderInverted = ci;
				}

				public int getDriveMotorID() {
					return driveMotorID;
				}

				public int getAngleMotorID() {
					return angleMotorID;
				}

				public double getAngleOffset() {
					return angleOffset;
				}

				public boolean isDriveMotorInverted() {
					return driveMotorInverted;
				}

				public boolean isAngleMotorInverted() {
					return angleMotorInverted;
				}

				public boolean isCANCoderIverted() {
					return cancoderInverted;
				}

				public int getCancoderID() {
					return cancoderID;
				}

			} // End ENUM SwerveModuleConstants
		}

		public class SysIdConstants {
			public static final double rampRate = 0.01;
			public static final double stepVoltage = 0.05;
			public static final double timeOut = Units.millisecondsToSeconds(5000);
		}

		public static final class Intake {
			public static final int INTAKE_MOTOR_CAN_ID = 51;
			// public static final boolean INTAKE_SENSOR_PHASE = false;
			public static final boolean INTAKE_INVERTED = false; // positive power - note in
			// public static final double INTAKE_NEUTRAL_DEADBAND = 0.001;
			// public static final int INTAKE_TIMEOUT = 30; //in ms
			public static final double INTAKE_NOTE_GRAB_POWER = 0.45;
			public static final double INTAKE_NOTE_FORWARD_POWER = 0.35;
			public static final double INTAKE_NOTE_SPEW_POWER = -0.35;

			public static final boolean GPM_SENSOR_PRESENT = true; // turn to TRUE when sensor will be configured
			public static final int GPM_SENSOR_SWITCH_DIO_PORT_NUMBER = 4;
		}
	}

	public static final class EnableCurrentLimiter {
		public static final boolean intake = true;
	}

	public static final class CurrentLimiter {
		public static int drive = 60;
		public static int intake = 0;
		public static int arm = 40;
		public static int shooter = 40;
	}
	public static class IMUConstants {
		public static final int kPigeonId = 40; //TODO: adjust the id according to the robot (C2025 is 40)

		// Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
		private static final Pigeon2Configuration pigeonConfigs = null;
	}

		public static final class OIConstants {
		public static final int driverControllerPort = 0;

		public static final int buttonBoxPort = 4;

		public static final int driverInterfaceSwitchButton = 1;

		public static final int robotCentricButton = 5; // XBOX L1 button

		public static final ControllerDeviceType driverInterfaceType = ControllerDeviceType.XBOX_ONEDRIVE;

		public static final int CALIBRATION_JOYSTICK_SLIDER_AXLE = 3;

		public static enum ControllerDeviceType {
			LOGITECH,
			PS5,
			XBOX, // RightJ F/B, LeftJ L/R, L2/R2 - rotation
			XBOX_ONEDRIVE // RIghtJ F/B/L/R, LeftJ - rotation
		}

		public static enum ControllerDevice {
			DRIVESTICK(
					0, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			// DRIVESTICK1,2,3 are used only for GPM calibration
			DRIVESTICK1(
					1, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			DRIVESTICK2(
					2, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			DRIVESTICK3(
					3, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			TURNSTICK( // Controls the rotation of the swervebot
					2, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			XBOX_CONTROLLER(
					5, // Port Number for Xbox controller
					ControllerDeviceType.XBOX,
					0.03, // deadband X for Xbox
					0.03, // deadband Y for Xbox //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
					0.03, // deadband Omega for Xbox
					false, // No cube controller configuration for Xbox yet
					false),

			XBOX_CONTROLLER_GPM(
					4, // Port Number for Xbox controller
					ControllerDeviceType.XBOX,
					0.03, // deadband X for Xbox
					0.03, // deadband Y for Xbox //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
					0.03, // deadband Omega for Xbox
					false, // No cube controller configuration for Xbox yet
					false);

			private ControllerDeviceType controllerDeviceType;
			private int portNumber;
			private double deadbandX;
			private double deadbandY;
			private double deadbandOmega;
			private boolean cubeControllerLeftStick;
			private boolean cubeControllerRightStick;

			ControllerDevice(int pn, ControllerDeviceType cdt, double dx, double dy, double dm, boolean ccL,
					boolean ccR) {
				this.portNumber = pn;
				this.controllerDeviceType = cdt;
				this.deadbandX = dx;
				this.deadbandY = dy;
				this.deadbandOmega = dm;
				this.cubeControllerLeftStick = ccL;
				this.cubeControllerRightStick = ccR;
			}

			public ControllerDeviceType getControllerDeviceType() {
				return controllerDeviceType;
			}

			public int getPortNumber() {
				return portNumber;
			}

			public double getDeadbandX() {
				return deadbandX;
			}

			public double getDeadbandY() {
				return deadbandY;
			}

			public double getDeadbandOmega() {
				return deadbandOmega;
			}

			public boolean isCubeControllerLeftStick() {
				return cubeControllerLeftStick;
			}

			public boolean isCubeControllerRightStick() {
				return cubeControllerRightStick;
			}
		}
		
	}

	public static final class GPMConstants{
		public static final class ClimberConstants {
			public static final int CLIMBER_MOTOR_CAN_ID = 52;

			//public static final double defautClimberMotorPower = 0.6;
			public static double climbUpPower = 0.1; //TODO: Change the value accordingly 
			public static final double climbDownPower = 0.3; 
			public static final boolean climberInverted = true;
		}

		public static final class IntakeConstants {

			public static final int INTAKE_ROLLERMOTOR_CAN_ID = 59;
			public static final boolean INTAKE_ROLLERMOTOR_INVERTED = true;
			
			public static final class IntakePIDConstants {

				public static final double kP = 0.02;
				public static final double kI = 0.000;
				public static final double kD = 2.0;
				public static final double kF = 0;
				public static final double kMaxOutput = 0.6;
				public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
				public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
				public static final int Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for
														// S-Curve (greater value yields greater smoothing).
				public static final double DefaultAcceptableError = 5; // Sensor units
				public static final double Izone = 500;
				public static final double PeakOutput = 0.5; // Closed Loop peak output
				public static final double NeutralDeadband = 0.001;
				public static final int periodMs = 10; // status frame period
				public static final int timeoutMs = 30; // status frame timeout
				public static final int closedLoopPeriod = 1; // 1ms for TalonSRX and locally connected encoder

				public static final double anglePIDTolerance = 0.5; // degree tolerance when rotating arm to angle using
																	// PID
			}

			public static final double rampRate = 0.25;
			// TODO: Check conversion factors; find the ones that work best with PID
			public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
			public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / 60;
			public static final double nominalVoltage = 12.0;
			public static final double coralIntakePower = 0.2; 
			public static final double coralShootingPowerL23 = 0.5; 
			public static final double coralShootingPowerL4 = 0.2; 
			public static final double algaeIntakePower = -0.7;
			public static final double algaeHoldPower = -0.2;
			public static final double algaeExpelPower = 0.2;
			public static final double algaeStallCurrent = 38.0;


			public static final class ReefFinderConstants{
				public static final int reefCANRangeID = 61; 
				public static final double newProximityThreshold = 10.0;
				public static final double newUpdateFrequency = 5.0;
				public static final double maxDistanceToTarget = 0.4;
				public static final double minDistanceToTarget = 0.2;
				public static final double reefFOVRangeX = 6.75;
				public static final double reefFOVRangeY = 6.75;
			}

			public static final class IntakeCoralCANRangeConstants{
				public static final int intakeCANRangeID = 60; 
				public static final double newProximityThreshold = 10.0;
				public static final double newUpdateFrequency = 10.0;
				public static final double maxDistanceToTarget = 0.015;
				public static final double minDistanceToTarget = 0.005;
				public static final double intakeFOVRangeX = 10;
				public static final double intakeFOVRangeY = 10;
			}
			
		}
		public static final class ElevatorConstants {
			public static final double elevatorMotorMetersPerRotation = 1.0;

			public static enum ElevatorHeights{ // meters off the ground for the piece placement
				Down(0.0),
				AlgaeReefLow(13.971),
				AlgaeReefHigh(24.367),
				AlgaeIntakeUp(0.0),
				ReefLevelOne(2.0),
				ReefLevelTwo(7.036113),
				ReefLevelThree(18.119141),
				ReefLevelFour(39.871),
				Barge(0.0),
				Processor(0.0),
				CoralIntake(0.0),
				MinimumChassisSpeedReductionHeight(1.0),
				MaxHeight(0.0);
				private double elevatorHeightForGamepiecePlacement;
				ElevatorHeights(double height) {
				  this.elevatorHeightForGamepiecePlacement = height;
				}
				public double getHeight() {
				  return elevatorHeightForGamepiecePlacement;
				}
			  }

			public static final int ELEVATOR_LEADERMOTOR_CAN_ID = 11; 
			public static final int ELEVATOR_FOLLOWERMOTOR_CAN_ID = 10; 
			public static final boolean ELEVATOR_MOTOR_INVERTED = true; 
			public static final int ELEVATOR_DOWN_LIMIT_SWITCH_DIO_PORT_NUMBER = 0;  
			public static final boolean IS_LIMIT_SWITCH_PRESENT = true; 
			public static final double holdingDutyCycleConstant = 0.04;

			public static class ElevatorPIDConstants {
				public static class PositionDutyCycleConstants {
					public static final double elevator_kP = 0.1;
					public static final double elevator_kI = 0.0;
					public static final double elevator_kD = 0.01;
					public static final double elevator_kV = 0.12;
				}

				public static class PositionVoltageConstants {
					public static final double elevator_kP = 0.1;
					public static final double elevator_kI = 0.0;
					public static final double elevator_kD = 0.01;
				}

				public static class MotionMagicDutyCycleConstants {
					public static final int slot = 0;
					public static final double elevator_kP = 0.64;
					public static final double elevator_kI = 0.0;
					public static final double elevator_kD = 0.0;
					public static final double MotionMagicCruiseVelocity = 75.0;
					public static final double motionMagicAcceleration = 150.0;
					public static final double motionMagicJerk = 1500.0;
				}

				public static class MotionMagicVoltageConstants {
					public static final int slot = 0;
					public static final double elevator_kP = 0.4;
					public static final double elevator_kI = 0.0;
					public static final double elevator_kD = 0.06;
					public static final double elevator_kS = 0.24;
					public static final double elevator_kV = 0.12;
					public static final double MotionMagicCruiseVelocity = 50.0;
					public static final double motionMagicAcceleration = 100.0;
					public static final double motionMagicJerk = 1000.0;
				}

				public static final double tolerance = 1.0;
			}
			public static final double zeroPositionAbsoluteEncoder = 0; //TODO: Measure on Robot
		}	

		public static final class ArmConstants {
			public static final int ARM_MOTOR_CAN_ID = 58;
			// public static final boolean INTAKE_SENSOR_PHASE = false;
			public static final boolean ARM_INVERTED = true; // positive power - note in

			public static final int THROUGHBORE_ENCODER_CAN_ID = 62;
			public static final double THROUGHBORE_ENCODER_ABSOLUTE_ZERO_VALUE = 0.635; // This position is NOT all the way back, but rather about 2 inches from it
			public static final double THROUGHBORE_ENCODER_BEYOND_ALL_THE_WAY_BACK = 0.85; // we never should be able to get to that position when rotation the arm BACKWARDS
			//public static final double MOTOR_ROTATIONS_PER_THROUGHBORE_ROTATIONS = (24.372070 - 0.060059)/(1.0 - 0.703 + 0.237); //TODO: CHECK ON ROBOT :)
			public static final double MOTOR_ROTATIONS_PER_THROUGHBORE_ROTATIONS = (16.65 + 3.2749) / (0.68 - 0.25); //TODO: CHECK ON ROBOT :)

			/**
			 * This ENUM contains arm positions for piece pickup and placement, in Kraken motor rotations from calibrated ZERO
			 * Calibrated ZERO is set via THROUUGHBORE ENCODER on the rotation arm shaft
			 */
			public static enum ArmPositions{ // position of the arm for the piece placement/pickup
				CoralIntake(-2.05),
				CoralCruise(1.0), // after coral intake - position, so the elevator can be safely raised
				AlgaeIntake(19.6),  
				//AlgaeIntake2(19.6),
				AlgaeRelease(10),  //TODO: Needs values from robot
				ReefLevelOne(-0.3),   //TODO: Needs values from robot
				ReefLevelTwo(0.0),   //TODO: Needs values from robot
				ReefLevelThree(0.0), //TODO: Needs values from robot
				ReefLevelFour(8.5),  //TODO: Needs values from robot
				Barge((1-0.703)*MOTOR_ROTATIONS_PER_THROUGHBORE_ROTATIONS),
				Processor((1.201-0.703)*MOTOR_ROTATIONS_PER_THROUGHBORE_ROTATIONS);
				private double armPositionForGamepiecePlacement;
				ArmPositions(double position) {
				  this.armPositionForGamepiecePlacement = position;
				}
				public double getPosition() {
				  return armPositionForGamepiecePlacement;
				}
			  }

			  public static enum ArmPower{ // position of the arm for the piece placement/pickup
				CoralIntakePower(0.5),
				CoralCruise(1.0), // after coral intake - position, so the elevator can be safely raised
				AlgaeIntakePower(-0.2),  
				AlgaeReleasePower(0.2);  //TODO: Needs values from robot
		
				private double armPower;
				ArmPower(double power) {
				  this.armPower = power;
				}
				public double getPower() {
				  return armPower;
				}
			  }

			public static final class ArmPIDConstants {
				public static class PositionDutyCycleConstants {
					public static final double arm_kP = 0.1;
					public static final double arm_kI = 0.0;
					public static final double arm_kD = 0.01;
					public static final double arm_kV = 0.12;
				}

				public static class PositionVoltageConstants {
					public static final double arm_kP = 0.1;
					public static final double arm_kI = 0.0;
					public static final double arm_kD = 0.01;
				}

				public static class MotionMagicDutyCycleConstants {
					public static final int slot = 0;
					public static final double arm_kP = 0.64; //0.64
					public static final double arm_kI = 0.0;
					public static final double arm_kD = 0.0;
					public static final double MotionMagicCruiseVelocity = 50.0; //75.0
					public static final double motionMagicAcceleration = 100.0; //150.0
					public static final double motionMagicJerk = 1000.0; //1500.0
				}

				public static class MotionMagicVoltageConstants {
					public static final int slot = 0;
					public static final double arm_kP = 0.4;
					public static final double arm_kI = 0.0;
					public static final double arm_kD = 0.06;
					public static final double arm_kS = 0.24;
					public static final double arm_kV = 0.12;
					public static final double MotionMagicCruiseVelocity = 50.0;
					public static final double motionMagicAcceleration = 100.0;
					public static final double motionMagicJerk = 1000.0;
				}

				public static final double tolerance = 3.0;
			}
		}
	}

	public static final class VisionHelperConstants {
		public static final double distanceBetweenReefPoles = Units.inchesToMeters(12.94); // page 162 https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf
		public static final double bumperWidth = Units.inchesToMeters(2.5);
		public static class RobotPoseConstants {
			public static Map<String, Pose2d> visionRobotPoses = new HashMap<String, Pose2d>();
			public static Map<Integer, String> tagNumberToKey = new HashMap<Integer, String>();
		}
	}

	public static final class LLVisionConstants {
		public static enum LLCamera {
			LLFRONT(
				"limelight-front"
			),

			LLBACK(
				"limelight-back"
			),

			LLLEFT(
				"limelight-fl"
			),

			LLRIGHT(
				"limelight-fr"
			);
			private String cameraname;
			LLCamera(String cn) {
				this.cameraname = cn;
			}
			public String getCameraName() {
				return cameraname;
			}
		}

	}

	public static final class AutoConstants {
		public static enum autoPoses {

			//

		}
	}
}
