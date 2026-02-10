package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;

public class VisionbutBetter {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    PhotonCamera leftCam;
    PhotonCamera rightCam;
    
    private static Transform3d rightCameraTransform = new Transform3d(.159,-.272,0.284988,new Rotation3d(0,0,Math.toRadians(15)));
    private static Transform3d leftCameraTransform = new Transform3d(.159,.272,0.284988,new Rotation3d(0,0,-Math.toRadians(16)));
    
    PhotonPoseEstimator leftEstimator;
    PhotonPoseEstimator rightEstimator;
    
    public VisionbutBetter() {
        leftCam = new PhotonCamera("leftCamera");
        rightCam = new PhotonCamera("rightCamera");
        leftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY,  rightCameraTransform);
        rightEstimator= new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, leftCameraTransform);
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        leftEstimator.setReferencePose(prevEstimatedRobotPose);
        return leftEstimator.update(leftCam.getAllUnreadResults().get(0));
    }
    
}