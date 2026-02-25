package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase{
    public TalonFX HoodMotor;
    public TurretVision turretVision;
    public String hoodState = "readyToShoot";
    public double targetHoodPos;
    public final MotionMagicVoltage hoodPosVolt = new MotionMagicVoltage(0).withSlot(1);

    public HoodSubsystem(){
        var hoodConfig = new TalonFXConfiguration();

        var slot1Configs = hoodConfig.Slot1;        
        slot1Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot1Configs.kV = 0.12;
        slot1Configs.kP = 4.8; // An error of 1 rps results in 0.11 V output
        slot1Configs.kI = 0.0; // no output for integrated error
        slot1Configs.kD = 0.1; // no output for error derivative

        var motionMagicConfigs = hoodConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

        turretVision = new TurretVision();

        HoodMotor = new TalonFX(Constants.Hood_ID);
        HoodMotor.getConfigurator().apply(hoodConfig, 0.050);
        HoodMotor.setPosition(0);
    }

    public void moveHood(){
        targetHoodPos = turretVision.getTurretDistance();
        if (targetHoodPos < 0) {
            targetHoodPos = 0;
        }
        HoodMotor.setControl(hoodPosVolt.withPosition(SmartDashboard.getNumber("hood target position", 0)));
    }

    public void forceHoodMove(double speed){
        HoodMotor.set(speed);
    }

    public String getHoodState(){
        return hoodState;
    }

    @Override
    public void periodic(){
        if (HoodMotor.getPosition().getValueAsDouble() <= targetHoodPos + 0.01 
        && HoodMotor.getPosition().getValueAsDouble() >= targetHoodPos - 0.01) {
            hoodState = "readyToShoot";
        }
        else {
            hoodState = "cantShoot";
        }
        SmartDashboard.putString("hood State", hoodState);
        SmartDashboard.putNumber("hood position", HoodMotor.getPosition().getValueAsDouble());
    }
}
