// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;


/** Add your docs here. */
public class MechanismConstants {

    
    public static final int kArmRightCanId = 1;
    public static final int kArmLeftCanId = 2;
    public static final int kLeftClimberCanId = 11;
    public static final int kRightClimberCanId = 12;
    public static final int kTopShooterCanId = 13;
    public static final int kBottomShooterCanId = 14; 
    public static final int kArmExtenderCanId = 15;
    public static final int kWristCanId = 16; 
    public static final int kIntakeCanId = 17; 
    public static final int kLaserCanId = 18; 
    public static final int kSparkPowerDistributionCanID = 19; 


    public static final double kShooterTopRPM = 7000;
    public static final int kShooterBotomRPM = 6500;
    
    public static final boolean kShooterTopMotorInverted = true;
    public static final boolean kShooterBotomMotorInverted = true;
    public static final int kShooterMotorCurrentLimit = 30; // amps
    public static final double kShooterP = 1;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0;
    public static final double kShooterMinOutput = -1;
    public static final double kShooterMaxOutput = 1;
    public static double shooterToleranceRPM = 50.0;


    public static final boolean kIntakeSparkMaxInverted = true;
    public static final double kIntakeP = 1;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;
    public static final double kIntakeMinOutput = -.25;
    public static final double kIntakeMaxOutput = .25;
    
    public static final boolean kWristSparkMaxInverted = true; 
    public static final double kWristP = 1;
    public static final double kWristI = 0;
    public static final double kWristD = 0;
    public static final double kWristMinOutput = -.25;
    public static final double kWristMaxOutput = .25;
    
    public static final double kIntakePickupPostion = 0;
    public static final double kIntakePickupRPM = 0;
    public static final double kIntakeShootPostion = 0;
    public static final double kIntakeShootRPM = 0;

    public static final double kClimberResetPosition = 0;
    public static final double kClimberClimbPosition = 90;
    public static final double kClimberGrabPosition = 100;

    public static final double kArmPickupPOS = 1;
    public static final double kArmExtenderPickupPOS = 1;

    public static final double kArmShootPOS = 1;
    public static final double kArmExtenderShootPOS = 1;

    //Home Postion   
    public static final double kArmHomePOS = 22;
    public static final double kArmExtenderHomePOS = 0;
    public static final double kWristHomePos = 75;

    //Move
    public static final double kArmMovePOS = 25;
    public static final double kArmExtenderMovePOS = 150;
    public static final double kWristMovePOS = 90;

    


}
