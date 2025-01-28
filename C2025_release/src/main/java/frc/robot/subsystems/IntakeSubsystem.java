// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.LocalTime;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimiter;
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.IntakeConstants;
import frc.robot.Constants.GPMConstants.IntakeConstants.IntakeMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.IntakeConstants.IntakePIDConstants;
import frc.robot.Constants.SwerveConstants.Intake;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  // NEO motors connected to Spark Max
  private SparkMax intakeMotor;

  private SparkClosedLoopController intakePIDController;
  private RelativeEncoder intakePIDEncoder;

  public static DigitalInput GPMsensors;
  private DigitalInput intakeDownLimitSwitch;

  public IntakeSubsystem() {

    // Check if need to initialize arm
    if (!EnabledSubsystems.intake) {
      return;
    }

    intakeMotor = new SparkMax(IntakeMotorConstantsEnum.ROLLERMOTOR.getIntakeMotorID(), MotorType.kBrushless);

    intakePIDController = intakeMotor.getClosedLoopController();

    // Set Arm encoders
    intakePIDEncoder = intakeMotor.getEncoder();

    // Main Motor; should not follow the other motor
    configureIntakeMotors(intakeMotor, intakePIDEncoder, intakePIDController, IntakeMotorConstantsEnum.ROLLERMOTOR, null);


    System.out.println("*** Intake initialized");

    if (Intake.GPM_SENSOR_PRESENT) {
      try {
        GPMsensors = new DigitalInput(Intake.GPM_SENSOR_SWITCH_DIO_PORT_NUMBER);
        System.out.println("*** Note sensor initialized");
      } catch (Exception e) {
        System.out.println("Unable to get note sensor value");
      }
    }

    
  }

  private void configureIntakeMotors(SparkMax motor,  RelativeEncoder encoder, SparkClosedLoopController p, IntakeMotorConstantsEnum c,
      SparkMax motorToFollow) {

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    //motor.restoreFactoryDefaults(); //restores the state of the motor to factory defaults
    motor.clearFaults();  //clears a fault that has occurred since the last time the faults were reset
    sparkMaxConfig.inverted(c.getIntakeMotorInverted()); //sets motor inverted if getArmMotorInverted() returns true

    sparkMaxConfig.idleMode(IdleMode.kBrake); //sets motor into brake mode
    //motor.setIdleMode(IdleMode.kCoast); 

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR);  //sets conversion between NEO units to necessary unit for positon
    sparkMaxConfig.apply(encoderConfig);

    motor.setCANTimeout(0); //sets up timeout

    sparkMaxConfig.voltageCompensation(IntakeConstants.nominalVoltage);  //enables voltage compensation for set voltage [12v]
   
    if (EnableCurrentLimiter.intake) {
      sparkMaxConfig.smartCurrentLimit(CurrentLimiter.arm); // sets current limit to 40 amps
    }
    
    sparkMaxConfig.openLoopRampRate(IntakeConstants.rampRate);  // sets the rate to go from 0 to full throttle on open loop
    sparkMaxConfig.closedLoopRampRate(IntakeConstants.rampRate);  // sets the rate to go from 0 to full throttle on open loop


    SignalsConfig signalsConfig = new SignalsConfig();

    // apply signals
    sparkMaxConfig.apply(signalsConfig);

    // ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    // // --- PID Setup
    // // set the PID sensor to motor encoder for hardware PID
    // closedLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // // set arm PID coefficients - LIFT
    // closedLoopConfig.p(IntakePIDConstants.kP);
    // closedLoopConfig.i(IntakePIDConstants.kI);
    // closedLoopConfig.d(IntakePIDConstants.kD);
    // closedLoopConfig.iZone(IntakePIDConstants.Izone);
    // //p.setFF(ArmPIDConstants.kF);
    // // kMaxOutput = 1 ; range is -1, 1
    // closedLoopConfig.outputRange(-IntakePIDConstants.kMaxOutput, IntakePIDConstants.kMaxOutput);

    // // Apply closed loop configuration
    // sparkMaxConfig.apply(closedLoopConfig);

    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public boolean isNoteInIntake() {
    return (!Intake.GPM_SENSOR_PRESENT) || !GPMsensors.get();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
