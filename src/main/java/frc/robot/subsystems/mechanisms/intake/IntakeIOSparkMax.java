// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.TimingBudget;
import frc.robot.subsystems.mechanisms.MechanismConstants;

/** Add your docs here. */
public class IntakeIOSparkMax implements IntakeIO{
    private final CANSparkFlex m_Wrist;
    private final CANSparkMax  m_Intake;
    private final AbsoluteEncoder wristEncoder;
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController wristController;
    private final SparkPIDController intakeController;
    private final PWMSparkMax blinken;

    private LaserCan laserCAN;


    public IntakeIOSparkMax() {
        m_Wrist = new CANSparkFlex(MechanismConstants.kWristCanId, MotorType.kBrushless);
        m_Intake = new CANSparkMax(MechanismConstants.kIntakeCanId, MotorType.kBrushless);
        blinken = new PWMSparkMax(9);

        wristEncoder = m_Wrist.getAbsoluteEncoder(Type.kDutyCycle);
        intakeEncoder =  m_Intake.getEncoder();
        
        wristEncoder.setInverted(true);    
        wristEncoder.setPositionConversionFactor(360);
        wristEncoder.setZeroOffset(333.3127069);
        SmartDashboard.putNumber("wristOffsetValue", wristEncoder.getZeroOffset());
    
        m_Wrist.setIdleMode(IdleMode.kBrake);       //MOTORBRAKE
        m_Intake.setIdleMode(IdleMode.kCoast);      //MOTORBRAKE
        m_Wrist.setInverted(false);
        m_Intake.setInverted(true);
        m_Wrist.setSmartCurrentLimit(40);
        m_Intake.setSmartCurrentLimit(40);
    
        intakeController = m_Intake.getPIDController();
        intakeController.setFeedbackDevice(intakeEncoder);
        intakeController.setP(.000008);
        intakeController.setI(0);
        intakeController.setD(0);
        intakeController.setIZone(0);
        intakeController.setFF(.0007);
        intakeController.setOutputRange(-.70,.70);

        wristController = m_Wrist.getPIDController();
        wristController.setFeedbackDevice(wristEncoder);
        wristController.setP(.00007);//NJ was .00028
        wristController.setI(0);
        wristController.setD(00004);
        wristController.setIZone(0);
        wristController.setFF(0.0001);
        wristController.setOutputRange(-1,1); //NJWAS -.70 .70
       // wristController.setPositionPIDWrappingEnabled(false);

        int smartMotionSlot = 0;
        wristController.setSmartMotionMaxVelocity(4000, smartMotionSlot); //NJWAS1200
        wristController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        wristController.setSmartMotionMaxAccel(2000, smartMotionSlot); //NJWAS700
        wristController.setSmartMotionAllowedClosedLoopError(3, smartMotionSlot);

        m_Wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_Wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_Wrist.setSoftLimit(SoftLimitDirection.kForward, 240);
        m_Wrist.setSoftLimit(SoftLimitDirection.kReverse, 5);
        



        m_Wrist.burnFlash();
        m_Intake.burnFlash();



        laserCAN = new LaserCan(MechanismConstants.kLaserCanId);

        try {
            laserCAN.setRangingMode(RangingMode.SHORT);
            laserCAN.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
        e.printStackTrace();
        }

 
        SmartDashboard.putBoolean("IntakeMode", (m_Intake.getIdleMode() == IdleMode.kBrake));
        SmartDashboard.putBoolean("WristMode", (m_Wrist.getIdleMode() == IdleMode.kBrake));
    } 
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeRPM = intakeEncoder.getVelocity();
        inputs.wristposition = wristEncoder.getPosition();
        inputs.intakeCurrent = m_Intake.getOutputCurrent();
        inputs.wristCurrent = m_Wrist.getOutputCurrent();
        inputs.wristOutput = m_Wrist.getAppliedOutput();
        inputs.wristVelocity = wristEncoder.getVelocity();
        inputs.lcMeasurement =  laserCAN.getMeasurement() == null ? 0 : laserCAN.getMeasurement().distance_mm;
        
        
    }
    @Override
    public void setIntakeMotors(double intakeRPM, double wristPostion) {
        intakeController.setReference(intakeRPM, CANSparkMax.ControlType.kVelocity);
        wristController.setReference(wristPostion, CANSparkMax.ControlType.kSmartMotion);
    }
 
    @Override
    public void setBlinken(double blinkenValue){
        blinken.set(blinkenValue);
    }


    public double getMeasurement() {
        return laserCAN.getMeasurement() == null ? 0 : laserCAN.getMeasurement().distance_mm;
    }

    
    @Override
    public void stop() {
        m_Wrist.stopMotor();
        m_Intake.stopMotor();
     }
}
