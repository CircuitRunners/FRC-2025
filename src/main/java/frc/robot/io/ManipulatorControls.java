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

    // public Trigger moveClawL4() {
    //     return y();
    // }

    public Trigger scoreL4Algae2() {
        return a();
    }
    
    public Trigger scoreL4() {
        return b();
    }
    
    public Trigger Algae1() {
        return y();
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
        return povUp();
    }

    public Trigger moveToL3() {
        return povLeft();
    }

    public Trigger moveToL2() {
        return povDown();
    }

    public Trigger moveToL1() {
        return povRight();
    }
}