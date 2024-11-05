// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive.*;
import frc.robot.subsystems.mechanisms.intake.IntakeIOSparkMax;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.Vision.PhotonVision;
import frc.robot.subsystems.Vision.PhotonVisionPoseEstimation;
import frc.robot.subsystems.Vision.ShooterAlignments;
import frc.robot.subsystems.mechanisms.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanisms.MechanisimControl;
import frc.robot.subsystems.mechanisms.MechanisimControl.State;
import frc.robot.subsystems.mechanisms.arm.ArmIOSparkMax;
import frc.robot.subsystems.mechanisms.arm.ArmSubsystem;
import frc.robot.subsystems.mechanisms.climber.ClimberIOSparkMax;
import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final MechanisimControl mechanisimControl;
  private final ShooterAlignments shooterAlignments;
  
 // private final ShooterAlignments shooterAlignments;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Trigger autoAimTrigger = new Trigger(this::runAutoAim);
  private final Trigger hasTagTrigger = new Trigger(this::hasTag);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                //new GyroIONavx(),
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset),
                new ModuleIOSparkMax(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset),
                new ModuleIOSparkMax(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset),
                new ModuleIOSparkMax(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset));
        shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
        intakeSubsystem = new  IntakeSubsystem(new IntakeIOSparkMax());
        armSubsystem = new ArmSubsystem(new ArmIOSparkMax());
        climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMax());
        shooterAlignments = new ShooterAlignments(drive, new PhotonVision(new PhotonVisionPoseEstimation()));
        mechanisimControl = new MechanisimControl(intakeSubsystem, shooterSubsystem, armSubsystem, climberSubsystem, shooterAlignments);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.kFrontLeftChassisAngularOffset),
                new ModuleIOSim(DriveConstants.kFrontRightChassisAngularOffset),
                new ModuleIOSim(DriveConstants.kBackLeftChassisAngularOffset),
                new ModuleIOSim(DriveConstants.kBackRightChassisAngularOffset));
        shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
        intakeSubsystem = new  IntakeSubsystem(new IntakeIOSparkMax());
        armSubsystem = new ArmSubsystem(new ArmIOSparkMax());
        climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMax());
        shooterAlignments = new ShooterAlignments(drive, new PhotonVision(new PhotonVisionPoseEstimation()));
        mechanisimControl = new MechanisimControl(intakeSubsystem, shooterSubsystem, armSubsystem, climberSubsystem, shooterAlignments);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
        intakeSubsystem = new  IntakeSubsystem(new IntakeIOSparkMax());
        armSubsystem = new ArmSubsystem(new ArmIOSparkMax());
        climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMax());
        shooterAlignments = new ShooterAlignments(drive, new PhotonVision(new PhotonVisionPoseEstimation()));
        mechanisimControl = new MechanisimControl(intakeSubsystem, shooterSubsystem, armSubsystem, climberSubsystem, shooterAlignments); 
        break;
    }

 
    configureNamedCommands();

   
    
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

 
  private void configureNamedCommands() {
     // Set up auto routines
    

    NamedCommands.registerCommand("PrepToShoot",
       new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.PREPARE_SHOOT)));
       
    NamedCommands.registerCommand("grabprep",
       new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.GRABPREP)));

       NamedCommands.registerCommand("Home",
       new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.HOME)));

       NamedCommands.registerCommand("Shoot",
       new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.SHOOT)));

       NamedCommands.registerCommand("Speaker",
       new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.SPEAKER)));

       NamedCommands.registerCommand("WingPrep",
       new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.WINGPREP)));

       NamedCommands.registerCommand("Pickup",
       new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.PICKUP)));

       NamedCommands.registerCommand("AmpShoot",
      new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.AMPSHOOT)));

      NamedCommands.registerCommand("FarShot",
       new InstantCommand(() -> mechanisimControl.setDesiredState(MechanisimControl.State.BLUE_LINE_SHOOT)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drive.drive(
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OIConstants.m_driverController.getRightX(), OIConstants.kDriveDeadband),
                !OIConstants.kdriveJoyButton.getRawButton(5),
                 OIConstants.kdriveJoyButton.getRawButton(1)),
            drive).withName("default drive command"));

      OIConstants.m_driverController.y().onTrue(Commands.runOnce(drive::setX, drive));
      
      OIConstants.m_driverController.rightBumper().onTrue(new InstantCommand(drive::zeroHeading));
        //OIConstants.m_driverController.b()
          //.onTrue(Commands.runOnce(
            //          () ->
              //            drive.resetOdometry(
                //              new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                  //    drive)
                  //.ignoringDisable(true));


//Mechanisim Control

      new JoystickButton(OIConstants.kauxController, 8) // HOME
            .onTrue( Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.HOME)));
      
      OIConstants.m_AuxXBox.rightBumper().onTrue(Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.HOME)));

      new JoystickButton(OIConstants.kauxController, 2) // PICKUP
            .onTrue(Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.PICKUP)));

      OIConstants.m_AuxXBox.rightTrigger().onTrue(Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.PICKUP)));              
      // new JoystickButton(OIConstants.kauxController, 6) // EJECT
      //       .onTrue(Commands.runOnce(
      //         () -> mechanisimControl.setDesiredState(MechanisimControl.State.EJECT)));

      new JoystickButton(OIConstants.kauxController, 1) // AMP
            .onTrue(Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.AMP)));

      OIConstants.m_AuxXBox.leftTrigger().onTrue(Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.AMP)));
     
      OIConstants.m_driverController.x()// AMPSHOOT 
            .onTrue(Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.AMPSHOOT)));


      new JoystickButton(OIConstants.kauxController, 4) // PREPARE_SHOOT
            .onTrue( Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.PREPARE_SHOOT)));

      OIConstants.m_AuxXBox.leftBumper().onTrue(Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.STORE)));


      new JoystickButton(OIConstants.kauxController, 3) // STORE
            .onTrue( Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.STORE)));

      OIConstants.m_driverController.rightTrigger() // SHOOT
            .onTrue( 
              Commands.runOnce( 
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.SHOOT)));
    
    OIConstants.m_driverController.leftTrigger() // CLOSESHOOT
            .onTrue( 
              Commands.runOnce( 
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.CLOSESHOOT)));

      OIConstants.m_driverController.y()// AUTO_AIM 
             .whileTrue(Commands.run(
               () -> shooterAlignments.periodic()));
      
      OIConstants.m_driverController.y()// AUTO_AIM 
             .whileFalse(Commands.run(
               () -> shooterAlignments.periodic()));

       OIConstants.m_driverController.y()
             .whileTrue(drive.run( 
               () -> drive.drive(
                 -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                 -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
                 shooterAlignments.setMotors(), 
                 !OIConstants.kdriveJoyButton.getRawButton(5), OIConstants.kdriveJoyButton.getRawButton(1))));
      
    //  OIConstants.m_driverController.y()
    //         .onTrue(Commands.runOnce(
    //          () -> mechanisimControl.setDesiredState(MechanisimControl.State.SMART_ANGLE)
    //          ));

  
      new JoystickButton(OIConstants.kauxController, 9) // POV Up Grab
           .onTrue( Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.GRAB)));

      new JoystickButton(OIConstants.kauxController, 10) // POV Up CLIMB
            .onTrue( Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.CLIMB)));
      
      new JoystickButton(OIConstants.kauxController, 13) // POV Up CLIMB Shoot
            .onTrue( Commands.sequence(
              Commands.runOnce(() -> mechanisimControl.setDesiredState(MechanisimControl.State.CLIMBSHOOT)),
              Commands.waitSeconds(1),
              Commands.runOnce(() -> mechanisimControl.setDesiredState(MechanisimControl.State.CLIMBSHOOT2))
              ));

      // new JoystickButton(OIConstants.kauxController, 5) // GRABPREP
      //       .onTrue( Commands.runOnce(
      //         () -> mechanisimControl.setDesiredState(MechanisimControl.State.GRABPREP)));

      new JoystickButton(OIConstants.kauxController, 7) // PARKCLIMBER
            .onTrue( Commands.runOnce(
              () -> mechanisimControl.setDesiredState(MechanisimControl.State.PARKCLIMBER)));
    
      new JoystickButton((OIConstants.kauxController), 5).onTrue(Commands.runOnce(
        ()-> mechanisimControl.setDesiredState(State.PASSING)));

      new JoystickButton(OIConstants.kauxController, 6).onTrue(Commands.runOnce(
        ()-> mechanisimControl.setDesiredState(State.SLOWSHOT)));
    
    OIConstants.m_AuxXBox.y().onTrue(Commands.runOnce(
        ()-> mechanisimControl.setDesiredState(State.SLOWSHOT)));
    OIConstants.m_AuxXBox.b().onTrue(Commands.runOnce(()-> mechanisimControl.setDesiredState(State.PREPARE_SHOOT)));
    
    autoAimTrigger.whileTrue(//Commands.runOnce(
        //()-> mechanisimControl.setDesiredState(State.PREPARE_SHOOT))
        //.andThen(
        autoTarget()
        //)
        //.until(this::readyToAutoShoot)
        //.andThen(Commands.runOnce(
        //()-> mechanisimControl.setDesiredState(State.SHOOT)))
        );
    hasTagTrigger.onTrue(Commands.runOnce(() -> intakeSubsystem.requestBlinken(.77)));
    hasTagTrigger.onFalse(Commands.runOnce(() -> intakeSubsystem.requestBlinken(.61)));
    SmartDashboard.putData(drive);







 //     new JoystickButton(OIConstants.kauxController, 13) // RESET BLINKIN//
 //           .onTrue( Commands.runOnce(
 //             () -> mechanisimControl.setDesiredState(MechanisimControl.State.RESETBLINKEN)));

   //   new POVButton(OIConstants.kauxController, 270)  // POV DRIVER Arm Manual Control 
   //         .onTrue( Commands.runOnce(
   //           () -> mechanisimControl.setDesiredState(MechanisimControl.State.MANUALARM)));


      
  
  }

  public Command whichAutoTarget() {
    boolean always = true;
    if (always) {
      return Commands.runOnce(
        ()-> mechanisimControl.setDesiredState(State.PREPARE_SHOOT))
        .andThen(autoTarget())
        .until(this::readyToAutoShoot)
        .andThen(Commands.runOnce(
        ()-> mechanisimControl.setDesiredState(State.SHOOT)));
    } else {
      return Commands.runOnce(
        ()-> mechanisimControl.setDesiredState(State.AUTO_SHOOT))
        .andThen(autofarTarget())
        .until(this::readyToFarAutoShoot)
        .andThen(Commands.runOnce(
        ()-> mechanisimControl.setDesiredState(State.SHOOT)));
    }
  }

  public Command autoTarget(){
      return
        new RunCommand(
          () -> drive.drive(
              autoYAxis(),
              autoXAxis(),
              -drive.targetYaw * .125 * .1,
              false,
              OIConstants.kdriveJoyButton.getRawButton(1)),
              drive).withName("auto aim command");
  }

    public Command autofarTarget(){
      return
        new RunCommand(
          
          () -> drive.drive(
              autoFarYAxis(),
              -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              -drive.targetYaw * .125 * .1,
              false,
              OIConstants.kdriveJoyButton.getRawButton(1)),
              drive).withName("auto aim command");
  }

  public double autoYAxis() {
    double yAxis = 0;
      double error = drive.targetPitch - 24.0;
      if (error > .25) {
        yAxis = .125 * error; 
        } else if (error < -.25) {
        yAxis = .125 * error;
        }
      yAxis = Math.sin(drive.targetYaw) * yAxis;
    
    return yAxis;
  }

  public double autoXAxis() {
    double xAxis = 0.0;
    double error = drive.targetPitch - 24.0;
        if (error > .25) {
        xAxis = .125 * error; 
        } else if (error < -.25) {
        xAxis = .125 * error;
        }
      xAxis = Math.cos(drive.targetYaw) * xAxis;
      
    return xAxis;
  }

    public double autoFarYAxis() {
      double yAxis = -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(), OIConstants.kDriveDeadband);
      double error = drive.targetPitch - 16.5;
      if (Math.abs(drive.targetYaw) < 2.5) {
         if (error > .25) {
          yAxis = .125 * error; 
         } else if (error < -.25) {
          yAxis = .125 * error;
         }

      }
      return yAxis;
  }


  public boolean runAutoAim(){
    if(OIConstants.kdriveJoyButton.getRawButton(1) && drive.targetVisible){
      return true;
    } else {
      return false;
    }
  }

  public boolean readyToAutoShoot() {
    if (Math.abs(drive.targetYaw) < 2.5 &&  Math.abs(drive.targetPitch - 24.0) < .25) {
      return true;
    } else {
      return false;
    }
  }

  public boolean hasTag() {
    if (drive.targetVisible && mechanisimControl.MachanismState() != State.PICKUP.toString()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean readyToFarAutoShoot() {
    if (Math.abs(drive.targetYaw) < 2.5 &&  Math.abs(drive.targetPitch - 16.5) < .25) {
      return true;
    } else {
      return false;
    }
  }


  public IntakeSubsystem getIntakeSubsystem(){
    return intakeSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem(){
    return shooterSubsystem;
  }

  public double desiredArmAngle() {
    return 22.5;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    
  }
}