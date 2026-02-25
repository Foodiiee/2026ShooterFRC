package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretMovement;
import frc.robot.subsystems.TurretVision;

public class TurretLeft extends Command {
    boolean stopSwitch;
    TurretVision m_turretVision;
    TurretMovement m_shooter; 

    public TurretLeft(TurretVision turretVision, TurretMovement shooter){
        m_turretVision = turretVision;
        addRequirements(turretVision);

         m_shooter = shooter;
        addRequirements(shooter);

    }

    @Override
    public void initialize(){
        stopSwitch = false;
        m_shooter.turnLeft(Constants.turnVoltage);
    }

    @Override
    public void execute(){
        // if the switch on the left side of the bot is being hit than set stopSwitch to true
        if(m_shooter.getLeftSwitch() == false){
            stopSwitch = true;
        }

    }

    

    @Override
    public void end(boolean interrupted){
        m_shooter.stopTurn();
        stopSwitch = false;
    }

    @Override
    public boolean isFinished(){
        // if stopSwitch is true stop command
        return stopSwitch;
    }

} 
