// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.mechanisms.MechanismConstants;


public class ShooterSubsystem  extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private boolean spinning = false;
    private double topShooterRPM = 0.0;
    private double bottomShooterRPM = 0.0;

    public ShooterSubsystem(ShooterIO io){
      //  System.out.println("[Init] Creating Shooter");
        this.io = io;

    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (DriverStation.isDisabled()) {
            stop();
        } else {
            io.setRPM(topShooterRPM, bottomShooterRPM);
        }
    
        
            Logger.recordOutput("Shooter/TopRPM", inputs.topShooterVelocityRPM);
            Logger.recordOutput("Shooter/BotomRPM", inputs.bottomShooterVelocityRPM);
        
    }

    public void requestRPM(double topShooterRPM, double bottomShooterRPM) {
         this.topShooterRPM = topShooterRPM;
        this.bottomShooterRPM = bottomShooterRPM;
      }
    
    
      public void stop() {
        topShooterRPM = 0;
        bottomShooterRPM = 0;
      }

      public double getTopSooterVelocity() {
        return inputs.topShooterVelocityRPM;
      }
    
      public double getBottomShooterVelocity() {
        return inputs.bottomShooterVelocityRPM;
      }
 
    @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.topShooterVelocityRPM - topShooterRPM) <= MechanismConstants.shooterToleranceRPM
        && Math.abs(inputs.bottomShooterVelocityRPM - bottomShooterRPM) <= MechanismConstants.shooterToleranceRPM
        && spinning;
  }
    

 
}
