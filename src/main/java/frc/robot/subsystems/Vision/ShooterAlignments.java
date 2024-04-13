package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drive.Drive;
//import frc.robot.subsystems.Drive.DriveConstants;


public class ShooterAlignments{
    private InterpolatingTreeMap<Double, Double> interpolateMap = new InterpolatingDoubleTreeMap();
    private PhotonVision photonVision;
    private Drive drive;
    private Rotation3d SpeakerAngle = new Rotation3d();
    private PIDController rotationController = new PIDController(.1,0,0);  //4 0 0

    public double armAngle;
    public boolean VhasTarget = false;
    public Pose2d Vpose = new Pose2d(0,0, new Rotation2d(0));

  

    
    
    public ShooterAlignments(Drive drive, PhotonVision photonVision){
       this.drive = drive;
       this.photonVision = photonVision;
    }


    public void periodic(){
        addValuesToMap();
        Optional<Pose2d> estimatedPose = photonVision.getEstimatedPose();
        if (estimatedPose.isPresent() && photonVision.getPoseAmbiguity()  && photonVision.getLatestResult().hasTargets())  {
            VhasTarget = true;
            Vpose = estimatedPose.get();
            double distanceToSpeaker = getDistanceToSpeaker(Vpose);
            Logger.recordOutput("ShooterAlignmets/Distance", distanceToSpeaker);
            Logger.recordOutput("ShooterAlignments/Pose", Vpose);
            armAngle = angleArmToSpeaker(distanceToSpeaker);
            SmartDashboard.putNumber("armangle", armAngle);
            setMotors();

        } else {
            VhasTarget = false;
            armAngle = 22.5;

        }
    
        Logger.recordOutput("ShooterAlignments/armAngle", armAngle);

    }

    public void addValuesToMap() {
        interpolateMap.put(1.82, 6.8);
        interpolateMap.put(2.9, 22.5);
    }



    public double setMotors(){ 
        rotationController.setSetpoint(headingToFaceSpeaker(Vpose));
        rotationController.setTolerance(.05);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        double rotvalue = rotationController.calculate(drive.getPose().getRotation().getRadians());
       
     //  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,rotvalue);
     //  drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

       Logger.recordOutput("ShooterAlignments/rotvalue", rotvalue);
       return rotvalue;
    }
       


       @AutoLogOutput(key = "ShooterAlignments/DistanceToSpeaker") 
       public double getDistanceToSpeaker(Pose2d Vpose) {
        double distance;
        Translation2d blueSpeaker = FieldConstants.Speaker.bluecenterSpeakerOpening;
        Translation2d redSpeaker = FieldConstants.Speaker.redcenterSpeakerOpening;
        double speakerX;
        double speakerY;        
        if (DriverStation.getAlliance().get() == Alliance.Blue){
            speakerX = blueSpeaker.getX();
            speakerY = blueSpeaker.getY();
        }
        else{
            speakerX = redSpeaker.getX();
            speakerY = redSpeaker.getY();
        }

        double distanceX = Vpose.getX() - speakerX;
        double distanceY = Vpose.getY() - speakerY;
        distance = Math.sqrt(Math.exp(distanceX) + Math.exp(distanceY));
        return distance;
    }

    
    public double headingToFaceSpeaker(Pose2d Vpose) {
        Translation2d blueSpeaker = FieldConstants.Speaker.bluecenterSpeakerOpening;
        Translation2d redSpeaker = FieldConstants.Speaker.redcenterSpeakerOpening;
        double speakerX;
        double speakerY;
        double offset = 0;
        if (DriverStation.getAlliance().get() == Alliance.Blue){
            speakerX = blueSpeaker.getX();
            speakerY = blueSpeaker.getY();
            offset = 180;
        }
        else{
            speakerX = redSpeaker.getX();
            speakerY = redSpeaker.getY();
            offset = 0;
        }
        double distanceX = Vpose.getX() - speakerX;
        double distanceY = Vpose.getY() - speakerY;
        double angle = Math.toDegrees(Math.atan2(distanceY, distanceX));
        double finalangle = angle + offset;
        if (finalangle > 180){
            finalangle = -(360 - finalangle);
        }

        Logger.recordOutput("ShooterAlignments/HeadingtoFaceSpeaker", Math.toDegrees(finalangle));
        return finalangle;

    }
    
    @AutoLogOutput(key = "Vision/angleArmToSpeaker")
    public double angleArmToSpeaker(double distance){
        double armAngle = interpolateMap.get(distance);
        return armAngle;
    }






/* 
public void visionAlignment(){
    if (photonVision.getLatestResult().hasTargets()){
    AprilTagAngle = new Rotation3d(0,0,photonVision.getYaw());
    Rotation2d RobotAngle = drive.getPose().getRotation();

    }
}
*/


}