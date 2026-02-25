// get an encoder
// get the values of the encoder and convert them into comparable units, degrees
// set a setpoint ex. 20 degrees
// use the (optional pid) to move from the current position to the setpoint

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretMovement extends SubsystemBase{
    private TalonFX leftLeader;
    private static DigitalInput RightSwitch = new DigitalInput(1);
    private static DigitalInput LeftSwitch = new DigitalInput(0);
    int leftTriggerPressed;

    public TurretMovement(){
        leftLeader = new TalonFX(31);  
        leftLeader.setVoltage(0);
        leftTriggerPressed = 0;

    }

    //When called it turns the motor tho the right
    public void turnRight(double voltage) {
    leftLeader.setVoltage(voltage);
    System.out.println("right switch: " + RightSwitch.get());
    }

    //When called it turns the motor to the left
    public void turnLeft(double voltage) {
    leftLeader.setVoltage(voltage*-1);
    System.out.println("left switch: " + LeftSwitch.get());
    }

    public void lockedOn(double voltage){
        leftLeader.setVoltage(voltage);
    }

    //When called it stops the motor
    public void stopTurn() {
        leftLeader.setVoltage(0);
        System.out.println("STOP MOVING");
    }

    //Returns the value of the right limit switch
    public boolean getRightSwitch(){
        return RightSwitch.get();
    }

    //Returns the value of the left limit switch
    public boolean getLeftSwitch(){
        return LeftSwitch.get();
    }

    // returns the stopButton to be true
    public boolean getStopCommand(){
        return Constants.stopbutton = true;
    }

}
