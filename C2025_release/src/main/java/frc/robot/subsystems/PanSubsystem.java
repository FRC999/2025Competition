// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimiter;
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.IntakeConstants;
import frc.robot.Constants.GPMConstants.PanConstants;

public class PanSubsystem extends SubsystemBase {
   private SparkMax panMotor;
   private SparkClosedLoopController panPIDController;
   private RelativeEncoder panPIDEncoder;
  /** Creates a new VelcroSubsystem. */
  public PanSubsystem() {
    if (!EnabledSubsystems.pan) {
      return;
    }

    panMotor = new SparkMax(PanConstants.PAN_MOTOR_CAN_ID, MotorType.kBrushless);
    panPIDController = panMotor.getClosedLoopController();

    // Set Arm encoders
    panPIDEncoder = panMotor.getEncoder();

    configureIntakeMotor(panMotor, panPIDEncoder, panPIDController);
  }

  private void configureIntakeMotor(SparkMax motor,  RelativeEncoder encoder, SparkClosedLoopController p) {

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    //motor.restoreFactoryDefaults(); //restores the state of the motor to factory defaults
    motor.clearFaults();  //clears a fault that has occurred since the last time the faults were reset
    sparkMaxConfig.inverted(PanConstants.VELCRO_MOTOR_INVERTED); //sets motor inverted if getArmMotorInverted() returns true

    sparkMaxConfig.idleMode(IdleMode.kBrake); //sets motor into brake mode
    //motor.setIdleMode(IdleMode.kCoast); 

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR);  //sets conversion between NEO units to necessary unit for positon
    sparkMaxConfig.apply(encoderConfig);

    motor.setCANTimeout(0); //sets up timeout

    sparkMaxConfig.voltageCompensation(IntakeConstants.nominalVoltage);  //enables voltage compensation for set voltage [12v]
   
    if (EnableCurrentLimiter.pan) {
      sparkMaxConfig.smartCurrentLimit(CurrentLimiter.pan); // sets current limit to 40 amps
    }
    
    sparkMaxConfig.openLoopRampRate(PanConstants.rampRate);  // sets the rate to go from 0 to full throttle on open loop
    sparkMaxConfig.closedLoopRampRate(PanConstants.rampRate);  // sets the rate to go from 0 to full throttle on open loop


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

    panMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void runPanMotor(double speed) {
    panMotor.set(speed);
  }

  public void stopPanMotor() {
    panMotor.set(0);
  }

  public double getOutputCurrent() {
    return panMotor.getOutputCurrent();
  }

  public boolean isPanHeld() {
    return getOutputCurrent() >= PanConstants.panStallCurrent;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
