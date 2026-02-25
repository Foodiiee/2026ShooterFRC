package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSubsystem extends SubsystemBase{
    public TalonFX ShooterMotorLeader;
    public TalonFX ShooterMotorFollower;
    public String shooterState = "cantShoot";
    public double targetRPS;
    final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0).withSlot(0);
   
    public FlywheelSubsystem(){
        var flywheelConfig = new Slot0Configs();
        flywheelConfig.kS = 0.1; // Add 0.1 V output to overcome static friction
        flywheelConfig.kV = SmartDashboard.getNumber("kV", 0.001); // A velocity target of 1 rps results in 0.12 V output
        flywheelConfig.kP = SmartDashboard.getNumber("kP", 0.004); // An error of 1 rps results in 0.11 V output
        flywheelConfig.kI = SmartDashboard.getNumber("kI", 0.0); // no output for integrated error
        flywheelConfig.kD = SmartDashboard.getNumber("kD", 0.0); // no output for error derivative
        
        ShooterMotorLeader = new TalonFX(Constants.Flywheel_Lead_ID);
        ShooterMotorFollower = new TalonFX(Constants.Flywheel_Follower_ID);
        ShooterMotorLeader.getConfigurator().apply(flywheelConfig);
        ShooterMotorFollower.getConfigurator().apply(flywheelConfig);
        ShooterMotorFollower.setControl(new Follower(ShooterMotorLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void setShooterState(){
        if (shooterState == "cantShoot") {
            shooterState = "windShooter";
        }
        else if (shooterState == "readyToShoot" || shooterState == "windShooter") {
            shooterState = "cantShoot";
        }
    }

    public void spinShooter(double targetRPS){
        System.out.println("targetRPS: " + targetRPS);
        ShooterMotorLeader.setControl(shooterVelocityVoltage.withVelocity(targetRPS).withFeedForward(targetRPS/10));
        this.targetRPS = targetRPS;
    }

    public double getShooterSpeed(){
        return ShooterMotorLeader.getVelocity().getValueAsDouble();
    }

    public String getShooterState(){
        return shooterState;
    }

    @Override
    public void periodic(){
        if (ShooterMotorLeader.getVelocity().getValueAsDouble() < targetRPS + 5 
         && ShooterMotorLeader.getVelocity().getValueAsDouble() > targetRPS - 5 && targetRPS > 0) {
            shooterState = "readyToShoot";
        }
        else if (targetRPS > 0) {
            shooterState = "windShooter";
        }
        SmartDashboard.putString("shooter state", shooterState);
        SmartDashboard.putNumber("shooter speed", ShooterMotorLeader.getVelocity().getValueAsDouble());
    }
}
