package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.lib.LimelightHelpers;
import frc.robot.subsystems.Drive;

public class AlignToReefTagRelative extends Command{
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private Drive swerve;
    private double tagID = -1;

    public static final double X_REEF_ALIGNMENT_P = 3.3;
	public static final double Y_REEF_ALIGNMENT_P = 3.3;
	public static final double ROT_REEF_ALIGNMENT_P = 0.058;

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
	public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

    public final int MAX_VELOCITY_METERS_PER_SECOND = 2;
    public final int MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2;
    
    
    public AlignToReefTagRelative(boolean isRightScore, Drive swerve) {
        xController = new PIDController(X_SETPOINT_REEF_ALIGNMENT, 0, 0);
        yController = new PIDController(Y_SETPOINT_REEF_ALIGNMENT, 0, 0);
        rotController = new PIDController(ROT_REEF_ALIGNMENT_P, 0, 0);
        this.isRightScore = isRightScore;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rotController.setSetpoint(ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);
        xController.setSetpoint(X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);
        yController.setSetpoint(isRightScore ? Y_SETPOINT_REEF_ALIGNMENT : -Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

        tagID = LimelightHelpers.getFiducialID("");
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
            this.dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose("");
            SmartDashboard.putNumber("x", positions[2]);

            double xSpeed = xController.calculate(positions[2]);
            SmartDashboard.putNumber("xspee", xSpeed);
            double ySpeed = -yController.calculate(positions[0]);
            double rotValue = -rotController.calculate(positions[4]);

            double clampedXSpeed = Math.max(-MAX_VELOCITY_METERS_PER_SECOND, Math.min(xSpeed, MAX_VELOCITY_METERS_PER_SECOND));
            double clampedYSpeed = Math.max(-MAX_VELOCITY_METERS_PER_SECOND, Math.min(ySpeed, MAX_VELOCITY_METERS_PER_SECOND));
            double clampedRotSpeed = Math.max(-MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Math.min(rotValue, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
            swerve.driveRobotCentric(new ChassisSpeeds(clampedXSpeed, clampedYSpeed, clampedRotSpeed));

            if (!rotController.atSetpoint() ||
                !yController.atSetpoint() ||
                !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            swerve.driveRobotCentric(new ChassisSpeeds(0, 0, 0));
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotCentric(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
        return this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) ||
            stopTimer.hasElapsed(POSE_VALIDATION_TIME);
    }
}
