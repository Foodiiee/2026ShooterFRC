package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.struct.PhotonTrackedTargetSerde;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class TurretVision extends SubsystemBase {
    private final PhotonPoseEstimator photonEstimator;
    private PhotonCamera turretCamera;
    PhotonPipelineResult result;
    PhotonUtils turretUtils;
    Pose2d estimateTurret2d;
    boolean hasTargets;

    double DistanceX;
    double DistanceY;
    double RedAllianceSqaured;

    double turretAngle;
    double turretDistance;
    double turretTargetAngle;

    Field2d field2d = new Field2d();

    public TurretVision(){
        // sets up turretCamera
        turretCamera = new PhotonCamera("Turret_Camera");
        photonEstimator = new PhotonPoseEstimator(Constants.kTagLayout, Constants.kRobotToCam);
    }

    // gets the if we have AprilTags in the vision or not
    public boolean getTags(){
        //System.err.println("getTags Works");
        result = turretCamera.getLatestResult();
        hasTargets = result.hasTargets();
        return hasTargets;
    }

    // gets the yaw of the april tag
    
    public double getYaw(){
        result = turretCamera.getLatestResult();
        hasTargets = result.hasTargets();
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getYaw();
        }
        else{
            return 0;
        }
    }

    // gets the robot pose based on two april tags or tries with one
    public Field2d getRobotPose(){
        if (result != null && result.hasTargets()){
            var cameraResult = result.getMultiTagResult();
            if (cameraResult != null && cameraResult.isEmpty() == false) {
                var fieldToCamera = cameraResult.get().estimatedPose.best;
                field2d.setRobotPose(new Pose2d(fieldToCamera.getX(), fieldToCamera.getY(), fieldToCamera.getRotation().toRotation2d()));

                // gets the distance between the bot x and the middle of the hub x
                DistanceX = Constants.BlueHubX - field2d.getRobotPose().getX();
                // gets the distance between the bot y and the middle of the hub y
                DistanceY = Constants.BlueHubY - field2d.getRobotPose().getY();
                // calculates the distance of the bot from the hub
                turretDistance = Math.sqrt(DistanceX*DistanceX + DistanceY*DistanceY);

                // calculates the angle we want to get to
                turretTargetAngle = Math.atan(DistanceX / DistanceY);

                // calculates the angle of the bot from the middle
                turretAngle = (fieldToCamera.getRotation().toRotation2d().getDegrees());
            } 
            else {
                field2d.setRobotPose(new Pose2d()); //Maybe not such a good idea
            }
        } else {
            field2d.setRobotPose(new Pose2d());
        }
        return field2d;
    }

    // when called gives you the turret distance calculated previously
    public double getTurretDistance(){
        return turretDistance;
    }

    // when called gives you the turret angle calculated previously
    public double getTurretAngle(){
        return turretAngle;
    }

    // when called gives you the targetTurretAngle calculated previously
    public double getTurretTargetAngle(){
        return turretTargetAngle;
    }

    @Override
    public void periodic() { 

    }
}