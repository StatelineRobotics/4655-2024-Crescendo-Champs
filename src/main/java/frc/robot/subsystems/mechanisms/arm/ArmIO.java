// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.mechanisms.MechanismConstants;


/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    class ArmIOInputs{
        public double armPosCurrent = 0.0;
        public double armExtenderPosCurrent = 0.0; 
        public double armRightOutput = 0.0;
        public double armLeftOutput = 0.0;
        public double armExtenderOutput = 0.0;  
        public double armPosSet = MechanismConstants.kArmHomePOS;
        public double armExtenderPosSet = MechanismConstants.kArmExtenderHomePOS;

    }
  
    /** Update inputs */
    default void updateInputs(ArmIOInputs inputs) {}
    
    default void setArmPositions(double armPosSet, double armExtenderPosSet) {}

    /** Stop Arm */
    default void stop() {}   

} 
