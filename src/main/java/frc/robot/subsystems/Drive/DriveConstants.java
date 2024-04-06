// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = (Math.PI * 1.3) ; // radians per second // WAS 2 * Math.PI
    public static final double KMaxLinearSpeed = Units.feetToMeters(14.5);

    public static final double kDirectionSlewRate = 10.0; // radians per second
    public static final double kMagnitudeSlewRate = 10.0; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 10.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    
    public static final double kTrackRadius = Units.inchesToMeters (Math.sqrt(Math.pow(kTrackWidth,2 ) + Math.pow(kWheelBase, 2)));//(29.34); //MUST CHANGE - ALSO IT'S IN METERS was 29.34 
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0; //Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0; //Math.PI
    public static final double kBackLeftChassisAngularOffset = 0; //0
    public static final double kBackRightChassisAngularOffset = 0; //-Math.PI / 2

    // SPARK MAX CAN IDs

    public static final int kRearRightTurningCanId = 3;
    public static final int kRearRightDrivingCanId = 4;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kFrontLeftTurningCanId = 7; 
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftTurningCanId = 10;
    public static final int kRearLeftDrivingCanId = 9;


    public static final Rotation2d kChassisAngularOffset = Rotation2d.fromDegrees(0);

    public static final int kPigeonCanId = 19;

    public static final boolean kGyroReversed = true;

    public static final class ModuleConstants {

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final boolean kDrivingMotorInverted = true;

        public static final double kDrivingMotorReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 2;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -.7;
        public static final double kTurningMaxOutput = 7;

        public static final double kAutoDrivingP = 0.9 ;
        public static final double kAutoDrivingI = 0;
        public static final double kAutoDrivingD = 0;

        public static final double kAutoTurningP = .9;
        public static final double kAutoTurningI = 0;
        public static final double kAutoTurningD = 0;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 30; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final HolonomicPathFollowerConfig follower = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(ModuleConstants.kAutoDrivingP, ModuleConstants.kAutoDrivingI, ModuleConstants.kAutoDrivingD), // Translation PID constants
                        new PIDConstants(ModuleConstants.kAutoTurningP, ModuleConstants.kAutoTurningI, ModuleConstants.kAutoTurningD), // Rotation PID constants
                        AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s ***MIGHT CHANGE***
                        kTrackRadius, // Drive base radius in meters. Distance from robot center to furthest module. ***MIGHT CHANGE***
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                );

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}
