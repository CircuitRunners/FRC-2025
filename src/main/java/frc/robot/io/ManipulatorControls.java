package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive;


public class ManipulatorControls extends CommandXboxController{
    
    public ManipulatorControls(int port){
        super(port);
    }

    //elevator controls
    public Trigger moveElevatorL4(){
        return povUp();
    }

    public Trigger moveElevatorL3(){
        return povLeft();
    }

    public Trigger moveElevatorL2(){
        return povRight();
    }

    public Trigger moveElevatorL1(){
        return povDown();
    }

    public Trigger moveElevatorBottom() {
        return leftStick(); //Might need to change double check this
    }

    public double elevatorManual() {
        return getLeftY();
    }

    // claw controls
    public Trigger moveClawHorizontal() {
        return x();
    }

    public Trigger moveClawL4() {
        return y();
    }

    public Trigger moveClawIntake() {
        return a();
    }

    public Trigger runRollersOut() {
        return rightBumper();
    }
    
    public Trigger runRollersIn() {
        return leftBumper();
    }
    
    public double clawManual() {
        return getRightY();
    }

    //overall controls
    public Trigger resetToIntake() {
        return start();
    }
    
    public Trigger moveToL4() {
        return y();
    }

    public Trigger moveToL3() {
        return x();
    }

    public Trigger moveToL2() {
        return b();
    }

    public Trigger moveToL1() {
        return a();
    }
}