package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexAndSpindexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexAndSpindexCommand extends Command{
    public IndexAndSpindexSubsystem InSSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public boolean IndexOverride = false;
    public boolean ForceSpin = false;
    
    public IndexAndSpindexCommand(IndexAndSpindexSubsystem InSSubsystem, ShooterSubsystem shooterSubsystem, boolean ForceSpin){
        this.InSSubsystem = InSSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.ForceSpin = ForceSpin;
        addRequirements(InSSubsystem);
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute() {
        if (ForceSpin == false) {
            if (shooterSubsystem.getShooterState() != "cantShoot" ) {
                IndexOverride = false;
            }
            else {
                IndexOverride = true;
            }
            if (IndexOverride == true) {
                shooterSubsystem.spinShooter(SmartDashboard.getNumber("flywheelSpeed", 0));
                IndexOverride = false;
            }
            else if (IndexOverride == false) {
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
