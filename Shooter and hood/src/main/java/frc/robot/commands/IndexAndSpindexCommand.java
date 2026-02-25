package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexAndSpindexSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class IndexAndSpindexCommand extends Command{
    public IndexAndSpindexSubsystem InSSubsystem;
    public FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    public HoodSubsystem hoodSubsystem = new HoodSubsystem();
    public boolean IndexOverride = false;
    public boolean ForceSpin = false;
    
    public IndexAndSpindexCommand(IndexAndSpindexSubsystem InSSubsystem, boolean ForceSpin){
        this.InSSubsystem = InSSubsystem;
        this.ForceSpin = ForceSpin;
        addRequirements(InSSubsystem);
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute() {
        if (ForceSpin == false) {
            if (flywheelSubsystem.getShooterState() == "cantShoot") {
                flywheelSubsystem.spinShooter(SmartDashboard.getNumber("flywheelSpeed", 0));
                hoodSubsystem.moveHood();
            }
            else if (flywheelSubsystem.getShooterState() == "readyToShoot") {
                InSSubsystem.moveFeeder();
            }
        }
        else {
            InSSubsystem.moveFeeder();
        }
    }
    @Override
    public void end(boolean interrupted) {
        InSSubsystem.stopFeeder();
    }
    @Override
    public boolean isFinished() {
        if (RobotContainer.driveController.x().getAsBoolean() == false && ForceSpin == false 
        || RobotContainer.operatorController.b().getAsBoolean() == false && ForceSpin == true) {
            return true;
        }
        else {
            return false;
        }
    }
}
