// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double intakeRPM = 0.0;
    public double intakeCurrent = 0.0;
    public double wristposition = 0.0;
    public double wristCurrent = 0.0;
    public double blinkenValue = 0.53;
    public double lcMeasurement;
    public double wristOutput;
    public double wristVelocity;
 
  }

  /** Update inputs */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Set Input speed and position */
  default void setIntakeMotors(double intakeRPMm, double wristPostion) {}

  /** Set Input speed and position */
  default void setBlinken(double blinkenValue) {}

  default void isIndexerLoaded(Boolean isIndexerLoaded) {}

  /** Stop intake */
  default void stop() {}
}
