package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IndexAndSpindexSubsystem extends SubsystemBase{
    public SparkMax indexMotor;
    public SparkMax spindexMotor;
    public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public HoodSubsystem hoodSubsystem = new HoodSubsystem();

    public IndexAndSpindexSubsystem(){
        indexMotor = new SparkMax(Constants.Index_ID, MotorType.kBrushless);
        spindexMotor = new SparkMax(Constants.Spindex_ID, MotorType.kBrushless);
    }

    public void stopFeeder(){
        indexMotor.set(0);
        spindexMotor.set(0);
    }

    public void moveFeeder(){
        indexMotor.set(0.1);
        spindexMotor.set(0.5);
    }

    @Override
    public void periodic() {
        if (hoodSubsystem.getHoodState() == "readyToShoot" 
        && shooterSubsystem.getShooterState() == "readyToShoot" 
        && RobotContainer.hasTarget.getAsBoolean() == false) {
            RobotContainer.driveController.setRumble(GenericHID.RumbleType.kBothRumble, 100);
        }
        else {
            RobotContainer.driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }
}
