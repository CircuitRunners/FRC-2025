package frc.robot;

import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class Vision {
    public static final record VisionMeasurement (Pose2d pose, double timestamp, Matrix<N3, N1> stdDev) {}
    public final PhotonCamera frontLeftCam = new PhotonCamera("frontLeft");
    public final PhotonCamera frontRightCam = new PhotonCamera("frontRight");

    public final PhotonPoseEstimator frontLeftPoseEstimator = new PhotonPoseEstimator(VisionConstants.fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.frontLeftCamTransform);
    public final PhotonPoseEstimator frontRightPoseEstimator = new PhotonPoseEstimator(VisionConstants.fieldLayout,PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.frontRightCamTransform);

    public final Consumer<VisionMeasurement> visionMeasurementConsumer;
    
    public Vision(Consumer<VisionMeasurement> visionMeasurementConsumer){
        this.visionMeasurementConsumer = visionMeasurementConsumer;
    }
    
    public EstimatedRobotPose[] run(double robotYaw){
            LimelightHelpers.setCameraPose_RobotSpace("limelight", 
            0.5,    // Forward offset (meters) GET CORRECT VALUES
            0.0,    // Side offset (meters) GET CORRECT VALUES
            0.5,    // Height offset (meters) GET CORRECT VALUES
            0.0,    // Roll (degrees) GET CORRECT VALUES
            30.0,   // Pitch (degrees) GET CORRECT VALUES
            0.0     // Yaw (degrees) GET CORRECT VALUES
        );
    

        LimelightHelpers.SetRobotOrientation("limelight", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate limeLightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        // visionMeasurementConsumer.accept(
        //     new VisionMeasurement(limeLightEstimate.pose, limeLightEstimate.timestampSeconds,VecBuilder.fill(.5, .5, 9999999))
        // );

        var frontLeftUpdate = frontLeftPoseEstimator.update(frontLeftCam.getAllUnreadResults().get(frontLeftCam.getAllUnreadResults().size()-1));
        var frontRightUpdate = frontRightPoseEstimator.update(frontRightCam.getAllUnreadResults().get(frontRightCam.getAllUnreadResults().size()-1));
        EstimatedRobotPose frontLeftEstimate = new EstimatedRobotPose(null, robotYaw, null, null);
        EstimatedRobotPose frontRightEstimate = new EstimatedRobotPose(null, robotYaw, null, null);
        if(frontLeftUpdate.isPresent()) {
            frontLeftEstimate = frontLeftUpdate.get();
            // visionMeasurementConsumer.accept( 
            //     new VisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, VecBuilder.fill(.9, .9, .9))
            // );
        }
    
        if(frontRightUpdate.isPresent()) {
            frontRightEstimate = frontRightUpdate.get();
            // visionMeasurementConsumer.accept( 
            //     new VisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, VecBuilder.fill(.9, .9, .9))
            // );
        }

        return new EstimatedRobotPose[] { frontLeftEstimate, frontRightEstimate};
    }
}