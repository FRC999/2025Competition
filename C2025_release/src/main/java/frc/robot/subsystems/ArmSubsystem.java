// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GPMConstants.ArmConstants;
import frc.robot.Constants.SwerveConstants.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
   private TalonFX armMotor; // Kraken
   private final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
   private RelativeEncoder armEncoder;
   private double armEncoderZero;
  
  public ArmSubsystem() {
    armMotor = new TalonFX(ArmConstants.ARM_MOTOR_CAN_ID);

    configureArmMotor();
  }

  private void configureArmMotor() {

    //intakeMotor.configFactoryDefault();
    armMotor.getConfigurator().apply(new TalonFXConfiguration()); // reset the motor to defaults
    armMotor.setSafetyEnabled(false);

    var motorconfigs = new MotorOutputConfigs();
    var talonFXConfigurator = armMotor.getConfigurator();
    talonFXConfigurator.apply(motorconfigs);
  }
  public void runIntake(double speed) {
    armMotor.set(speed);
  }

  public void stopIntake() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
