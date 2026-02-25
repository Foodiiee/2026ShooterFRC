package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelCommand extends Command{
    public FlywheelSubsystem shooterSubsystem;
    public double targetRPS;

    public FlywheelCommand(FlywheelSubsystem shooterSubsystem, double targetRPS){
        this.shooterSubsystem = shooterSubsystem;
        this.targetRPS = targetRPS;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setShooterState();
        /* 
         * also make the default wind up for flywheel be controller op, but if you try to shoot and it
         * is not spinning then start spinning it.
         */
    }

    @Override
    public void execute(){
        switch (shooterSubsystem.getShooterState()) {
            case "windShooter":
                shooterSubsystem.spinShooter(SmartDashboard.getNumber("flywheelSpeed", targetRPS));
                break;
            case "cantShoot":
                shooterSubsystem.spinShooter(0);
                break;
        }
    }

    @Override
    public void end(boolean isInterupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
