// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io; 
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double intakeRPM = 0.0;
  private double wristPostion = 0.0;
  private double blinkenValue = 0.53;
  

  public IntakeSubsystem(IntakeIO io) {
  //  System.out.println("[Init] Creating Intake");
    this.io = io;
   
  }



  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

 
    if (DriverStation.isDisabled()) {
      stop();
    } else {
        io.setIntakeMotors(intakeRPM, wristPostion);
        io.setBlinken(blinkenValue);       
    }

   

  }
  
  public void requestIntake(double intakeRPM, double wristPostion) {
    this.intakeRPM = intakeRPM;
    this.wristPostion = wristPostion;
 }

   public void requestBlinken(double blinkenValue) {
    this.blinkenValue = blinkenValue;
   
 }

//  public boolean isRingLoaded() {
 //   return IntakeIO.isRingLoaded();
//  }

   private void stop() {
    intakeRPM = 0;
    wristPostion = 0;
    blinkenValue = 1525;
    io.stop();
  }

   public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }

    @AutoLogOutput(key = "Intake/HasNote")
    public boolean HasNote() {
    return (inputs.lcMeasurement < 60);
  }
  
    @AutoLogOutput(key = "Intake/OkToEject")
  public boolean OkToEject() {
    return (Math.abs(inputs.wristposition - 50) > 4);
  }

   @AutoLogOutput(key = "Intake/OkToSpeeker")
  public boolean OkToSpeeker() {
    return (inputs.wristposition > 160);
  }

  @AutoLogOutput(key = "Intake/OkIntake")
  public boolean OkIntake() {
    return (Math.abs(inputs.wristposition - 22) < 20);
  }


}
