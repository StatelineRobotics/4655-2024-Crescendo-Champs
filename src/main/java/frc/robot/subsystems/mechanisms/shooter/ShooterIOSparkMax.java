// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.subsystems.mechanisms.MechanismConstants;


/** Add your docs here. */
public class ShooterIOSparkMax implements ShooterIO {
    private CANSparkFlex m_TopShooter;
    private CANSparkFlex  m_BottomShooter;
    private RelativeEncoder topShooterEncoder;
    private RelativeEncoder bottomShooterEncoder;
    private SparkPIDController topShooterController;
    private SparkPIDController bottomShooterController;

    public ShooterIOSparkMax(){
        m_BottomShooter = new CANSparkFlex(MechanismConstants.kBottomShooterCanId, MotorType.kBrushless);
        m_TopShooter = new CANSparkFlex(MechanismConstants.kTopShooterCanId, MotorType.kBrushless);

        bottomShooterEncoder =  m_BottomShooter.getEncoder();
        topShooterEncoder =  m_TopShooter.getEncoder();

       
        m_BottomShooter.setIdleMode(IdleMode.kCoast);
        m_TopShooter.setIdleMode(IdleMode.kCoast);
        m_BottomShooter.setInverted(false);
        m_TopShooter.setInverted(true);
        m_BottomShooter.setSmartCurrentLimit(60);
        m_TopShooter.setSmartCurrentLimit(60);
        
        bottomShooterController = m_BottomShooter.getPIDController();
        bottomShooterController.setFeedbackDevice(bottomShooterEncoder);
        bottomShooterController.setP(0.0006);
        bottomShooterController.setI(0);
        bottomShooterController.setD(0);
        bottomShooterController.setIZone(0);
        bottomShooterController.setFF(0.000155);
        bottomShooterController.setOutputRange(-1, 1);

        topShooterController = m_TopShooter.getPIDController();
        topShooterController.setFeedbackDevice(topShooterEncoder);
        topShooterController.setP(0.0006);
        topShooterController.setP(0);
        topShooterController.setP(0);
        topShooterController.setIZone(0);
        topShooterController.setFF(0.000155);
        topShooterController.setOutputRange(-1, 1);

        m_BottomShooter.burnFlash();
        m_TopShooter.burnFlash();

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topShooterVelocityRPM = topShooterEncoder.getVelocity();
        inputs.bottomShooterVelocityRPM = bottomShooterEncoder.getVelocity();
    
       
    }

    @Override
    public void setTopShooterRPM(double rpm) {
        if (rpm > 100) { 
            topShooterController.setReference(rpm, CANSparkBase.ControlType.kVelocity); //5600
        }
        else {
            m_TopShooter.set(0);
        }    
    }
    
     @Override
    public void setBottomShooterRPM(double rpm) {
        if (rpm > 100) { 
            bottomShooterController.setReference(rpm, CANSparkBase.ControlType.kVelocity); //5900
        }
        else {
            m_BottomShooter.set(0);
        }
    }
 

    @Override
    public void stop() {
        m_TopShooter.stopMotor();
        m_BottomShooter.stopMotor();
     }

}


