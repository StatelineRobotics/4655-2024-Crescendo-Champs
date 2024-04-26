// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.ShooterAlignments;
import frc.robot.subsystems.mechanisms.arm.ArmSubsystem;
import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;


public class MechanisimControl extends SubsystemBase {
 
  public enum State {
    PREPARE_SHOOT,
    SHOOT,
    PICKUP,
    HOME,
    CLIMB,
    CLIMBSHOOT,
    GRAB,
    AMP,
    AMPSHOOT,
    EJECT,
    STORE,
    SPEAKERSHOOT,
    SPEAKER,
    WINGPREP,
    GRABPREP,
    PARKCLIMBER,
    AUTO_AIM,
    CLOSESHOOT,
    SMART_ANGLE,
    PASSING,
    BLUE_LINE_SHOOT,
    CLIMBSHOOT2,
  }

  public boolean AmpShoot;
  private State currentState = State.HOME;
 

  //@Getter private State currentState = State.IDLE;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final ShooterAlignments shooterAlignments;

  /** Creates a new MechanisimControl. */
  public MechanisimControl(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, ClimberSubsystem climberSubsystem, ShooterAlignments shooterAlignments) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.armSubsystem = armSubsystem;
    this.climberSubsystem = climberSubsystem;
    this.shooterAlignments = shooterAlignments; 
  }

  @Override
  public void periodic() {
    
    switch (currentState) {
      case HOME -> {
          if (armSubsystem.OkToHome()){ 
            intakeSubsystem.requestIntake(0, 50);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(37, 0);
            intakeSubsystem.requestBlinken(0.53);
          } else{
            intakeSubsystem.requestIntake(0, 50);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(38, 115);
            intakeSubsystem.requestBlinken(0.53);
          }
          AmpShoot = false;
          break;
       }

      case EJECT -> {
          if (armSubsystem.OkToHome()){ 
            if (intakeSubsystem.OkToEject()) {
              intakeSubsystem.requestIntake(-600, 60);
            } else {
              intakeSubsystem.requestIntake(0, 60);
            }
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(37, 0);
            intakeSubsystem.requestBlinken(0.53);
            } else {
            intakeSubsystem.requestIntake(0, 60);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(38, 115);
            intakeSubsystem.requestBlinken(0.53);
            }
            AmpShoot = false;
        break;
       } 
  
      case AMP -> {
        AmpShoot = true;
        intakeSubsystem.requestIntake(0, 155);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(77, 40);
        intakeSubsystem.requestBlinken(0.53);
        break;
       }

      case AMPSHOOT -> {
        intakeSubsystem.requestIntake(600, 155);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(77, 40);
        intakeSubsystem.requestBlinken(0.53);
        break;
       } 


       case SPEAKER -> {
        if (intakeSubsystem.OkToSpeeker()){ 
          armSubsystem.requestArmPosition(13, 0); //WAS 15
        } else {
          armSubsystem.requestArmPosition(37, 0 );
        }
        
        intakeSubsystem.requestIntake(0, 237);
        shooterSubsystem.requestRPM(5600, 5900);
        intakeSubsystem.requestBlinken(0.53);
        AmpShoot = false;
        break;
       }


       case WINGPREP -> {
        if (intakeSubsystem.OkToSpeeker()){ 
          armSubsystem.requestArmPosition(24, 0);
        } else {
          armSubsystem.requestArmPosition(37, 0 );
        }
        
        intakeSubsystem.requestIntake(0, 232);
        shooterSubsystem.requestRPM(5600, 5900);
        intakeSubsystem.requestBlinken(0.53);
        AmpShoot = false;
        break;
       }


      case SPEAKERSHOOT -> {
        AmpShoot = false;
        intakeSubsystem.requestIntake(600, 232);
        shooterSubsystem.requestRPM(5600, 5900);
        armSubsystem.requestArmPosition(6, 0);
        intakeSubsystem.requestBlinken(0.53);
        break;
       } 

      case PICKUP -> {
          if (armSubsystem.OkToPickup()){ 
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(23, 127);
              if(intakeSubsystem.HasNote() && armSubsystem.extendsFar() && intakeSubsystem.OkIntake()){
                intakeSubsystem.requestIntake(0, 22);
                intakeSubsystem.requestBlinken(-.05);
                currentState = State.STORE;
              } else {
                intakeSubsystem.requestIntake(500 , 22);
                intakeSubsystem.requestBlinken(0.53);
              }

          } else {
            intakeSubsystem.requestIntake(0, 32);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(38, 115);
            intakeSubsystem.requestBlinken(0.53); 
          }
          AmpShoot = false;
          break;
        }

      case STORE -> {
        intakeSubsystem.requestIntake(0,227);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(22.5, 0 );
        climberSubsystem.requestClimberPosition(144);
         if(intakeSubsystem.HasNote()){
          intakeSubsystem.requestBlinken(-.05);
        }else{
          intakeSubsystem.requestBlinken(0.53);
        }
        AmpShoot = false;
        break;
      }

        case PREPARE_SHOOT -> {
        intakeSubsystem.requestIntake(0,232);
        shooterSubsystem.requestRPM(5600, 5900);
        armSubsystem.requestArmPosition(22.5, 10);
        intakeSubsystem.requestBlinken(0.53);
        break;
      }


      case SHOOT -> {
 //         if(AmpShoot){
 //         intakeSubsystem.requestIntake(600, 158);
 //         intakeSubsystem.requestBlinken(0.53);
 //         } else {
           intakeSubsystem.requestIntake(-500,232);
           intakeSubsystem.requestBlinken(0.53);
 //         }
      break;
      }
      
      case AUTO_AIM -> {
        if (shooterAlignments.VhasTarget){
          intakeSubsystem.requestBlinken(-0.09);
        } else {
          intakeSubsystem.requestBlinken(0.61);
        }
        double armAngle = shooterAlignments.armAngle;
        intakeSubsystem.requestIntake(0,232);
        shooterSubsystem.requestRPM(5600, 5900);
        armSubsystem.requestArmPosition(armAngle, 10);
        break;
    }

      case CLIMB -> {
        if (armSubsystem.OkToClimbWrist()){
          intakeSubsystem.requestIntake(0,120);
        } else {
          intakeSubsystem.requestIntake(0,120);
        }
        
        shooterSubsystem.requestRPM(0, 0);
          if (climberSubsystem.ClimberOkToReach()) {
            armSubsystem.requestArmPosition(94, 95);
          }
          else {

            
            armSubsystem.requestArmPosition(124, 0);
          }
        climberSubsystem.requestClimberPosition(0);
        intakeSubsystem.requestBlinken(-0.57);
        break;
      }

      case CLIMBSHOOT -> {
        intakeSubsystem.requestIntake(500,120);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(94, 95);
        climberSubsystem.requestClimberPosition(0);
        intakeSubsystem.requestBlinken(-.57);
        break;
      }

      case CLIMBSHOOT2 -> {
        intakeSubsystem.requestIntake(500,120);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(90, 95);
        climberSubsystem.requestClimberPosition(0);
        intakeSubsystem.requestBlinken(-.57);
        break;
      }

      case GRAB -> {
        intakeSubsystem.requestIntake(0,47 );
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(123,0);
        climberSubsystem.requestClimberPosition(195);
        intakeSubsystem.requestBlinken(-.57);
        break;
      }

      case GRABPREP -> {
        climberSubsystem.requestClimberPosition(79);
        intakeSubsystem.requestBlinken(-.57);
        break;
      }

      case PARKCLIMBER -> {
        climberSubsystem.requestClimberPosition(-10);
        intakeSubsystem.requestBlinken(-.57);
        break;
      }
        case CLOSESHOOT -> {
        intakeSubsystem.requestIntake(0,232);
        shooterSubsystem.requestRPM(5600, 5900);
        armSubsystem.requestArmPosition(6.5, 10);
        intakeSubsystem.requestBlinken(0.53);
        break;
      }

        case SMART_ANGLE -> {
        
        double armAngle = SmartDashboard.getNumber("Arm Angle", 22.5);
        intakeSubsystem.requestIntake(0,232);
        shooterSubsystem.requestRPM(5600, 5900);
        armSubsystem.requestArmPosition(armAngle, 10);
        intakeSubsystem.requestBlinken(0.53);
        break;
        }


        case PASSING -> {
        intakeSubsystem.requestIntake(0,232);
        shooterSubsystem.requestRPM(5600/1.75, 5900/1.75);
        armSubsystem.requestArmPosition(22.5, 10);
        intakeSubsystem.requestBlinken(0.53);
        break;
      }

        case BLUE_LINE_SHOOT -> {
        intakeSubsystem.requestIntake(0,232);
        shooterSubsystem.requestRPM(5600, 5900);
        armSubsystem.requestArmPosition(33.5, 10);
        intakeSubsystem.requestBlinken(0.53);
        break;
      }
      


    }


    
  }
  
   

  @AutoLogOutput(key = "MechanisimControl/ReadyToShoot")
  public boolean atShootingSetpoint() {
    return (currentState == State.PREPARE_SHOOT || currentState == State.SHOOT)
        //&& ArmSubsystem.atSetpoint()
        && shooterSubsystem.atSetpoint();
  }
  

  @AutoLogOutput(key = "MechanisimControl/ReadyToStore")
  public boolean atReadyToSTore() {
    return (currentState == State.PICKUP || currentState == State.STORE)
        && intakeSubsystem.HasNote();
  }

  @AutoLogOutput(key = "MechanisimControl/DesieredState")
  public String MachanismState(){
    String machanismState = currentState.toString();
    return machanismState;
  }

  public void setDesiredState(State desiredState) {
//    if (currentState == State.PICKUP && intakeSubsystem.HasNote()) {
//     desiredState = State.STORE;
//    }
        currentState = desiredState;
    
  }

  

  
}
