package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HoodSubsystem;

public class HoodCommand extends Command{
    public HoodSubsystem hoodSubsystem;
    public boolean HoodOverride;
    public double targetHoodPos;
    
    public HoodCommand(HoodSubsystem hoodSubsystem, boolean HoodOverride, double targetHoodPos){
        this.hoodSubsystem = hoodSubsystem;
        this.HoodOverride = HoodOverride;
        this.targetHoodPos = targetHoodPos;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (HoodOverride) {
            hoodSubsystem.forceHoodMove(targetHoodPos);
        }
        else {
            hoodSubsystem.moveHood(SmartDashboard.getNumber("hood target position", 0));
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (RobotContainer.hasTarget.getAsBoolean() == true && HoodOverride == false) {
            return true;
        }
        return false;
    }
}
