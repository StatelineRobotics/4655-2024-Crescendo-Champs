// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Drive.DriveConstants.ModuleConstants;
import frc.robot.util.CANSpark;
import frc.robot.util.CANSpark.Motor;

public class ModuleIOSparkMax implements ModuleIO{
  private final CANSpark m_drivingSparkMax;
  private final CANSpark m_turningSparkMax;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private Rotation2d m_chassisAngularOffset;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private SwerveModuleState optimizedState = new SwerveModuleState(0.0, new Rotation2d());

  public ModuleIOSparkMax(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new Motor(CANSpark.Controller.MAX, drivingCANId)
      .idleMode(ModuleConstants.kDrivingMotorIdleMode)
      .currentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
      .inverted(ModuleConstants.kDrivingMotorInverted)
      .setRelativeConversionFactors(ModuleConstants.kDrivingEncoderPositionFactor, ModuleConstants.kDrivingEncoderVelocityFactor)
      .configure();
    m_turningSparkMax = new Motor(CANSpark.Controller.MAX, turningCANId)
      .idleMode(ModuleConstants.kTurningMotorIdleMode)
      .currentLimit(ModuleConstants.kTurningMotorCurrentLimit)
      .inverted(ModuleConstants.kTurningEncoderInverted)
      .hasAbsoluteEncoder(true)
      .setAbsoluteConversionFactors(ModuleConstants.kTurningEncoderPositionFactor, ModuleConstants.kTurningEncoderVelocityFactor)
      .configure();

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingSparkMax.setRelativeFeedbackDevice(m_drivingPIDController);
    m_turningSparkMax.setAbsoluteFeedbackDevice(m_turningPIDController);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    
    m_drivingSparkMax.burn();
    m_turningSparkMax.burn();
    


    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    m_chassisAngularOffset = Rotation2d.fromRadians(chassisAngularOffset);
    m_desiredState.angle = new Rotation2d(m_turningSparkMax.getAbsolutePosition());
    m_drivingSparkMax.resetEncoders();
  }

  @Override
  public void resetEncoders() {
    m_drivingSparkMax.resetEncoders();
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
    correctedDesiredState.angle = state.angle.plus(m_chassisAngularOffset);

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningSparkMax.getAbsolutePosition()));

    optimizedState = optimizedDesiredState; 

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    
    if (Math.abs(state.speedMetersPerSecond) < 0.05) {
      stopDrive();
      return;
    }
    else {
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    }

    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = state;
  
  }

  public void stopDrive() {
    m_drivingSparkMax.set(0);
  }

  @Override
  public SwerveModuleState getOptimizedState() {
    return optimizedState;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    if (m_chassisAngularOffset.getRadians() == 0) {
      Logger.recordOutput("Control", m_turningPIDController.getP());
    }

    inputs.drivePositionMeters = m_drivingSparkMax.getRelativePosition();
    inputs.driveVelocityMetersPerSec = m_drivingSparkMax.getRelativeVelocity();
    
    inputs.turnAbsolutePosition = Rotation2d.fromRadians(m_turningSparkMax.getAbsolutePosition()).minus(m_chassisAngularOffset);
    inputs.turnVelocityRadPerSec = m_turningSparkMax.getAbsoluteVelocity();
  }
}
