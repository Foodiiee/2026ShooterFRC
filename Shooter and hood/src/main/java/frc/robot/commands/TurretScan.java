package frc.robot.commands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretMovement;
import frc.robot.subsystems.TurretVision;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class TurretScan extends Command {
    TurretVision m_turretVision;
    TurretMovement m_shooter;
    boolean TurningRight = true;
    boolean hasTargets = false;
    double yaw;
    double distance;
    boolean stopLockedOn = false;
    int targetPose;
    Optional<EstimatedRobotPose> estimatedturretPose;
    Pose3d turretPose3d;
    Pose2d turretPose2d;
    double timestampSeconds;
    List<PhotonTrackedTarget> targeList;
    Field2d field2d;
    double turretTargetAngle;
    double turretAngle;

    public TurretScan(TurretVision turretVision, TurretMovement shooter){
        m_turretVision = turretVision;
        addRequirements(turretVision);

         m_shooter = shooter;
        addRequirements(shooter);

        turretPose2d = new Pose2d();
        field2d = new Field2d();
    }

    @Override
    public void initialize() {
        TurningRight = true;
        hasTargets = false;
        Constants.stopbutton = false;
        stopLockedOn = false;

    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("TurningRight", TurningRight);
         System.err.println("execute works");
        //get if the robot is seeing the april tag
        hasTargets = m_turretVision.getTags();
        // set the yaw to what the yaw of the april tag is


        if(hasTargets == false){
            
            // if TurningRight = true turn the turret to the right
            if(TurningRight == true){
                m_shooter.turnRight(Constants.turnVoltage);
                 System.err.println("Turn Right == true works");
            }

            // if TurningRight = false turn the turret to the left
            if(TurningRight == false){
                m_shooter.turnLeft(Constants.turnVoltage);
                 System.err.println("Turn Right == false works");
            }

            // if the left Switch is being pressed set TurningRight to true
            if(m_shooter.getLeftSwitch() == false){
                TurningRight = true;
                 System.err.println("getLeftSwitch works");
            }

            // if the right Switch is being pressed set TurningRight to false
            if(m_shooter.getRightSwitch() == false){
                TurningRight = false;
                 System.err.println("getRightSwitch works");

            }
        }
        else{
            field2d = m_turretVision.getRobotPose();

            // gets the turret angle relative to the field
            turretAngle = m_turretVision.getTurretAngle();
            // gets the angle we want to be at to be facing the hub
            turretTargetAngle = m_turretVision.getTurretTargetAngle();

            distance = m_turretVision.getTurretDistance();

            SmartDashboard.putNumber("turretDistance", distance);
            SmartDashboard.putNumber("turretPoseX", field2d.getRobotPose().getX());
            SmartDashboard.putNumber("turretPoseY", field2d.getRobotPose().getY());
            SmartDashboard.putNumber("turretRotation", field2d.getRobotPose().getRotation().getDegrees());
            SmartDashboard.putNumber("targetAngle", turretTargetAngle);
            SmartDashboard.putNumber("turretAngle", turretAngle);
        
            // if left or right switch is pressed while we see a target set stopLockedOn to true
            if(m_shooter.getLeftSwitch() == false || m_shooter.getRightSwitch() == false){
                stopLockedOn = true;
                System.out.println("stopLockedOn " + stopLockedOn);
            }

        //Setting the voltage of the motor to the yaw of the target multiplied by 5
            m_shooter.lockedOn((turretTargetAngle - turretAngle)/45*0.6);
            System.out.println(Constants.turnVoltage);

        
        System.out.println("hasTargets" + hasTargets);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopTurn();
        System.err.println(Constants.turnVoltage);
    }



    @Override
    public boolean isFinished() {
        // if rightTrigger is pressed or stopLocked = true then end command
        if(Constants.stopbutton == true || stopLockedOn == true){
            return true;
        }
        return false;
    }
    
}
