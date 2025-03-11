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
import frc.robot.Constants.GPMConstants.VelcroConstants;

public class VelcroSubsystem extends SubsystemBase {
   private SparkMax velcroMotor;
  /** Creates a new VelcroSubsystem. */
  public VelcroSubsystem() {
    if (!EnabledSubsystems.intake) {
      return;
    }

    velcroMotor = new SparkMax(VelcroConstants.VELCRO_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.inverted(VelcroConstants.VELCRO_MOTOR_INVERTED); //sets motor inverted if getArmMotorInverted() returns true
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    
  }
  public void runVelcroMotor(double speed) {
    velcroMotor.set(speed);
  }

  public void stopVelcroMotor() {
    velcroMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
