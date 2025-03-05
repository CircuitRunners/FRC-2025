package frc.robot;

import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.VisionConstants;

public class Vision {
    public static final record VisionMeasurement (Pose2d pose, double timestamp, Matrix<N3, N1> stdDev) {}
    // private final PhotonCamera frontCam = new PhotonCamera("front");
    // private final PhotonCamera backCam = new PhotonCamera("back");

    private final PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(VisionConstants.fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.frontCamTransform);
    // private final PhotonPoseEstimator backPoseEstimator = new PhotonPoseEstimator(VisionConstants.fieldLayout,PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.backCamTransform);

    private final Consumer<VisionMeasurement> visionMeasurementConsumer;
    
    public Vision(Consumer<VisionMeasurement> visionMeasurementConsumer){
        this.visionMeasurementConsumer = visionMeasurementConsumer;
    }
    
    public PoseEstimate run(Rotation2d robotYaw){
        //     LimelightHelpers.setCameraPose_RobotSpace("limelight-cr", 
        //     0,    // Forward offset (meters) GET CORRECT VALUES
        //     0.0,    // Side offset (meters) GET CORRECT VALUES
        //     0.927,    // Height offset (meters) GET CORRECT VALUES
        //     0.0,    // Roll (degrees) GET CORRECT VALUES
        //     0,   // Pitch (degrees) GET CORRECT VALUES
        //     180     // Yaw (degrees) GET CORRECT VALUES
        // );
    

        LimelightHelpers.SetRobotOrientation("limelight", robotYaw.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate limeLightEstimate;
        // if (DriverStation.getAlliance().equals(Alliance.Red)) {
        //     limeLightEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
        // } else {
        //     limeLightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // }
        limeLightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        System.out.println(limeLightEstimate.pose);
        return limeLightEstimate;
        
        
        // var frontUpdate = frontPoseEstimator.update(frontCam.getAllUnreadResults().get(frontCam.getAllUnreadResults().size()-1));
        // var backUpdate = backPoseEstimator.update(backCam.getAllUnreadResults().get(backCam.getAllUnreadResults().size()-1));
    
        // if(frontUpdate.isPresent()) {
        //     var estimate = frontUpdate.get();
        //     visionMeasurementConsumer.accept( 
        //         new VisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, VecBuilder.fill(.9, .9, .9))
        //     );
        // }
    
        // if(backUpdate.isPresent()) {
        //     var estimate = backUpdate.get();
        //     visionMeasurementConsumer.accept( 
        //         new VisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, VecBuilder.fill(.9, .9, .9))
        //     );
        // }
    }
}