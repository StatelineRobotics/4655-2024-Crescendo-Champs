// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.mechanisms.MechanismConstants;

/** Add your docs here. */
public class ClimberIOSparkMax implements ClimberIO {
    private CANSparkMax m_LeftClimber;
    private CANSparkMax  m_RightClimber;
    private RelativeEncoder rightClimberEncoder;
    private RelativeEncoder leftClimberEncoder;
    private SparkPIDController rightClimberController;
    private SparkPIDController leftClimberController;
    private SparkLimitSwitch rightClimberLimitSwitch;
    private SparkLimitSwitch leftClimberLimitSwitch;

    // PID coefficients
    
   

    public ClimberIOSparkMax(){
        m_LeftClimber = new CANSparkMax(MechanismConstants.kLeftClimberCanId, MotorType.kBrushless);
        m_RightClimber = new CANSparkMax(MechanismConstants.kRightClimberCanId, MotorType.kBrushless);

       // m_LeftClimber.restoreFactoryDefaults();
        //m_RightClimber.restoreFactoryDefaults();

        rightClimberEncoder =  m_RightClimber.getEncoder();
        leftClimberEncoder =  m_LeftClimber.getEncoder();

        // double kP = 0.00079; 
        // double kI = 0;
        // double kD = 0; 
        // double kIz = 0; 
        // double kFF = .00004; 
        // double kMaxOutput = .5; 
        // double kMinOutput = -.5;
        // double maxRPM = 5700;
        // double maxVel = 1500; 
        // double maxAcc = 1000;
        // double minVel = 0;

       
        m_LeftClimber.setIdleMode(IdleMode.kBrake);     //MOTORBRAKE
        m_RightClimber.setIdleMode(IdleMode.kBrake);    //MOTORBRAKE
        m_LeftClimber.setInverted(false);
        m_RightClimber.setInverted(false);
        m_LeftClimber.setSmartCurrentLimit(50);
        m_RightClimber.setSmartCurrentLimit(50);
        
        rightClimberLimitSwitch = m_RightClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        leftClimberLimitSwitch = m_LeftClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        
        rightClimberController = m_RightClimber.getPIDController();
        rightClimberController.setFeedbackDevice(rightClimberEncoder);
    //    rightClimberController.setP(kP);
    //    rightClimberController.setI(kI);
    //    rightClimberController.setD(kD);
    //    rightClimberController.setIZone(kIz);
    //    rightClimberController.setFF(kFF);
    //    rightClimberController.setOutputRange(kMinOutput,  kMaxOutput);
    //    int smartMotionSlot1 = 0;
    //    rightClimberController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot1);
    //    rightClimberController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot1);
    //    rightClimberController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot1);
       

        leftClimberController = m_LeftClimber.getPIDController();
        leftClimberController.setFeedbackDevice(leftClimberEncoder);
        // leftClimberController.setP(kP);
        // leftClimberController.setI(kI);
        // leftClimberController.setD(kD);
        // leftClimberController.setIZone(kIz);
        // leftClimberController.setFF(kFF);
        // leftClimberController.setOutputRange(kMinOutput,  kMaxOutput);
        // int smartMotionSlot2 = 0;
        // leftClimberController.setSmartMotionMaxVelocity(maxRPM, smartMotionSlot2);
        // leftClimberController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot2);
        // leftClimberController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot2);

        m_LeftClimber.burnFlash();
        m_RightClimber.burnFlash();

        SmartDashboard.putBoolean("ClimbLeftMode", (m_LeftClimber.getIdleMode() == IdleMode.kBrake));
        SmartDashboard.putBoolean("ClimbLeftMode", (m_RightClimber.getIdleMode() == IdleMode.kBrake));        
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.rightClimberPosition = rightClimberEncoder.getPosition();
        inputs.leftClimberPosition = leftClimberEncoder.getPosition();
        inputs.rightClimberCurrent = m_RightClimber.getOutputCurrent();
        inputs.leftClimberCurrent = m_LeftClimber.getOutputCurrent();


        

        if(rightClimberLimitSwitch.isPressed()) {
            rightClimberEncoder.setPosition(0);
        }

        if(leftClimberLimitSwitch.isPressed()) {
            leftClimberEncoder.setPosition(0);
        }

    }

    @Override
    public void setclimberMotors(double climberPosition) {
       leftClimberController.setReference(climberPosition, CANSparkMax.ControlType.kSmartMotion);
       rightClimberController.setReference(climberPosition, CANSparkMax.ControlType.kSmartMotion);
    }


    @Override
    public void stop() {
        m_LeftClimber.stopMotor();
        m_RightClimber.stopMotor();
     }
    

}
