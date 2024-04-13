
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import frc.robot.subsystems.mechanisms.MechanismConstants;

public class ArmIOSparkMax implements ArmIO {

    private CANSparkMax m_ArmRight;
    private CANSparkMax  m_ArmLeft;
    private CANSparkMax  m_ArmExtender;
    private AbsoluteEncoder armEncoder;
    private RelativeEncoder armExtenderEncoder;
    
    private SparkPIDController armController;
    private SparkPIDController armExtenderController;
    private SparkLimitSwitch extendLimitSwitch;

    public ArmIOSparkMax(){
        m_ArmRight = new CANSparkMax(MechanismConstants.kArmLeftCanId, MotorType.kBrushless);
        m_ArmLeft = new CANSparkMax(MechanismConstants.kArmRightCanId, MotorType.kBrushless);
        m_ArmExtender = new CANSparkMax(MechanismConstants.kArmExtenderCanId, MotorType.kBrushless);
         
        m_ArmRight.restoreFactoryDefaults();
        m_ArmLeft.restoreFactoryDefaults();
        m_ArmExtender.restoreFactoryDefaults();


        armEncoder =  m_ArmRight.getAbsoluteEncoder(Type.kDutyCycle);
        armEncoder.setInverted(true);
        armEncoder.setPositionConversionFactor(360);

        armExtenderEncoder = m_ArmExtender.getEncoder();  
   
        m_ArmRight.setIdleMode(IdleMode.kBrake);    //MOTORBRAKE
        m_ArmLeft.setIdleMode(IdleMode.kBrake);     //MOTORBRAKE
        m_ArmExtender.setIdleMode(IdleMode.kCoast); //MOTORBRAKE
        m_ArmRight.setInverted(false);
        m_ArmLeft.setInverted(false);
        m_ArmExtender.setInverted(true);
        m_ArmRight.setSmartCurrentLimit(40);
        m_ArmLeft.setSmartCurrentLimit(40);
        m_ArmExtender.setSmartCurrentLimit(20);
        m_ArmLeft.follow(m_ArmRight);  
    
        int smartMotionSlot = 0;

        armController = m_ArmRight.getPIDController();
        armController.setP(.010);
        armController.setI(0);
        armController.setD(0);
        armController.setIZone(0);
        armController.setFF(.000655);
        armController.setOutputRange(-.25,.50); 
        armController.setFeedbackDevice(armEncoder);
        armController.setSmartMotionMaxVelocity(50, smartMotionSlot);
        armController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        armController.setSmartMotionMaxAccel( 20, smartMotionSlot);
        armController.setSmartMotionAllowedClosedLoopError(1, smartMotionSlot);
        

        armExtenderController = m_ArmExtender.getPIDController();
        armExtenderController.setFeedbackDevice(armExtenderEncoder);
        armExtenderController.setP(.0008);
        armExtenderController.setP(0);
        armExtenderController.setP(0);
        armExtenderController.setIZone(0);
        armExtenderController.setFF(0.0025);
        armExtenderController.setOutputRange(-1,1);
        armExtenderController.setSmartMotionMaxVelocity(4000, smartMotionSlot); //NJWAS1300
        armExtenderController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        armExtenderController.setSmartMotionMaxAccel(2000, smartMotionSlot);  //NJWAS 600
        armExtenderController.setSmartMotionAllowedClosedLoopError(1, smartMotionSlot);
        extendLimitSwitch = m_ArmExtender.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        extendLimitSwitch.enableLimitSwitch(true);
        armExtenderEncoder.setPosition(0);
        m_ArmExtender.enableSoftLimit(SoftLimitDirection.kForward,true);
        m_ArmExtender.setSoftLimit(SoftLimitDirection.kForward, 130);

        m_ArmRight.burnFlash();
        m_ArmLeft.burnFlash();
        m_ArmExtender.burnFlash();
    }

     @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPosCurrent = armEncoder.getPosition();
        inputs.armExtenderPosCurrent = armExtenderEncoder.getPosition();
        inputs.armRightOutput = m_ArmRight.getAppliedOutput();
        inputs.armLeftOutput = m_ArmLeft.getAppliedOutput();
        inputs.armExtenderOutput = m_ArmExtender.getAppliedOutput();
        
    
          //if(extendLimitSwitch.isPressed()) {
          //  armExtenderEncoder.setPosition(0);
        //}
    }
   
    
     @Override
    public void setArmPositions(double armPosSet, double armExtenderPosSet) {
        armController.setReference(armPosSet, CANSparkMax.ControlType.kPosition);
        armExtenderController.setReference(armExtenderPosSet, CANSparkMax.ControlType.kSmartMotion);

           
    }
    
    @Override
    public void stop() {
        m_ArmRight.set(0);
        m_ArmLeft.set(0);
        m_ArmExtender.set(0);
     }

  


}
