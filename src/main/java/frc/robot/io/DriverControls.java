package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;


public class DriverControls extends CommandXboxController{
    
    public DriverControls(int port){
        super(port);
    }

    public double driveForward(){
        double input = MathUtil.applyDeadband(-getLeftY(), DriverConstants.stickDeadband);
        return input; 
    }

    public double driveStrafe(){
        double input = MathUtil.applyDeadband(-getLeftX(), DriverConstants.stickDeadband);
        return input;
    }

    public double driveRotation(){
        return MathUtil.applyDeadband(-getRightX(), DriverConstants.stickDeadband);
    }

    public Trigger resetGyro(){
        return start();
    }

    public Trigger robotRelative(){
        return leftTrigger();
    }

    public Trigger increaseLimit(){
        return rightBumper();
    }

    public Trigger decreaseLimit(){
        return leftBumper();
    }

    public Trigger toAmp(){
        return a();
    }

    public Trigger aimAtSpeaker(){
        return rightTrigger();
    }

    public Trigger robotMoveForward(){
        return povUp();
    }

    public Trigger robotMoveBack(){
        return povDown();
    }

    public Trigger robotMoveLeft(){
        return povLeft();
    }

    public Trigger robotMoveRight(){
        return povRight();
    }

    public Trigger toggleSysIdMode(){
        return y();
    }

    public Trigger toSource(){
        return b();
    }
}