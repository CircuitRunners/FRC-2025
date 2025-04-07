package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

import java.awt.Desktop;
import java.text.FieldPosition;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase{
    public static final double leftAlignmentX = .2435737274077523; //meters
    public static final double leftAlignmentY = 0.275;
    public static final double rightAlignmentX = .2435737274077523;
    public static final double rightAlignmentY = 0.623;
    public static final double thetaAlignment = -Math.PI/2; //degrees
    public static double maxAlignmentDistance = 1.5;

    public static final double xTolerance = .05;
    public static final double yTolerance = .05;
    public static final double thetaTolerance = .05;

    public static class FieldPositions {
        //Blue
        public static final Pose2d L17 = new Pose2d(4.019, 2.913, new Rotation2d(Math.toRadians(150)));
        public static final Pose2d L18 = new Pose2d(3.357, 3.829, new Rotation2d(Math.toRadians(90)));
        public static final Pose2d L19 = new Pose2d(3.75, 5.100, new Rotation2d(Math.toRadians(30)));
        public static final Pose2d L20 = new Pose2d(5.142, 5.263, new Rotation2d(Math.toRadians(-30)));
        public static final Pose2d L21 = new Pose2d(5.669, 3.664, new Rotation2d(Math.toRadians(-90)));
        public static final Pose2d L22 = new Pose2d(5.235, 2.941, new Rotation2d(Math.toRadians(210)));

        public static final Pose2d R17 = new Pose2d(4.328, 2.764, new Rotation2d(Math.toRadians(150)));
        public static final Pose2d R18 = new Pose2d(3.304, 3.510, new Rotation2d(Math.toRadians(90)));
        public static final Pose2d R19 = new Pose2d(3.439, 4.923, new Rotation2d(Math.toRadians(30)));
        public static final Pose2d R20 = new Pose2d(4.657, 5.436, new Rotation2d(Math.toRadians(-30)));
        public static final Pose2d R21 = new Pose2d(5.779, 4.379, new Rotation2d(Math.toRadians(-90)));
        public static final Pose2d R22 = new Pose2d(5.483, 3.261, new Rotation2d(Math.toRadians(210)));

        //Red
        public static final Pose2d L6 = new Pose2d(13.65, 2.92, new Rotation2d(Math.toRadians(210)));
        public static final Pose2d L7 = new Pose2d(14.32, 4.00, new Rotation2d(Math.toRadians(-90)));
        public static final Pose2d L8 = new Pose2d(13.72, 5.12, new Rotation2d(Math.toRadians(-30)));
        public static final Pose2d L9 = new Pose2d(12.45, 5.16, new Rotation2d(Math.toRadians(30)));
        public static final Pose2d L10 = new Pose2d(11.78, 4.08, new Rotation2d(Math.toRadians(90)));
        public static final Pose2d L11 = new Pose2d(12.38, 2.96, new Rotation2d(Math.toRadians(150)));

        public static final Pose2d R6 = new Pose2d(14.14, 3.20, new Rotation2d(Math.toRadians(210)));
        public static final Pose2d R7 = new Pose2d(14.32, 4.57, new Rotation2d(Math.toRadians(-90)));
        public static final Pose2d R8 = new Pose2d(13.22, 5.40, new Rotation2d(Math.toRadians(-30)));
        public static final Pose2d R9 = new Pose2d(11.95, 4.87, new Rotation2d(Math.toRadians(30)));
        public static final Pose2d R10 = new Pose2d(11.78, 3.51, new Rotation2d(Math.toRadians(90)));
        public static final Pose2d R11 = new Pose2d(12.87, 2.67, new Rotation2d(Math.toRadians(150)));
        public static final Pose2d BARGE_START = new Pose2d(7.449, 6.134, new Rotation2d(Math.toRadians(-30)));
        public static final Pose2d PROCESSOR_START = new Pose2d(7.449, 6.134, new Rotation2d(Math.toRadians(-30)));
        public static final List<Pose2d> kLeftReefPoses=Arrays.asList(
        L17, L18, L19, L20, L21, L22, L6, L7, L8, L9, L10, L11
        );
        public static final List<Pose2d> kRightReefPoses = Arrays.asList(
        R17, R18, R19, R20, R21, R22, R6, R7, R8, R9, R10, R11
        );

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4);
        public static final Translation2d RED_REEF_CENTER = new Translation2d(13, 4);


        public static final int [] kReefIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
        public static boolean isReefID(int id) {
        for(int i: kReefIDs) {
            if(id==i) return true;
        }
        return false;
        }

        public static final Pose2d BLUE_LEFT_CORAL_STATION_PICKUP = new Pose2d(new Translation2d(1.2,7), Rotation2d.fromDegrees(120));
        public static final Pose2d BLUE_CLIMB_AREA = new Pose2d(new Translation2d(7.638,6.174), Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_PROCESSOR = new Pose2d(new Translation2d(6.332,.52), Rotation2d.fromDegrees(-90));
    }


    public static AprilTagFieldLayout fieldLayout;

    private PhotonCamera reefCamera;
    private Optional<Pose2d> lastCalculatedDist;
    private PhotonPoseEstimator poseEstimator;
    private int latestID;
    private Pose2d reefDstPose;

    private PhotonCameraSim reefCameraSim;
    private VisionSystemSim visionSim;
    private SimCameraProperties prop = new SimCameraProperties().setAvgLatencyMs(.1).setCalibration(1080,720,Rotation2d.fromDegrees(71.4));

    StructPublisher<Pose2d> reefTagDisp = NetworkTableInstance.getDefault()
        .getStructTopic("SmartDashboard/Subsystem/Vision/RobotToTag", Pose2d.struct).publish();
    StructPublisher<Pose2d> estimatedCaemraPose = NetworkTableInstance.getDefault()
        .getStructTopic("SmartDashboard/Subsystem/Vision/estimatedCameraPose", Pose2d.struct).publish();

    public Vision() {

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        reefCamera = new PhotonCamera("ReefCamera");
        // TODO : Set the camera transform
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY,
            new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
        lastCalculatedDist = Optional.empty();

        reefCameraSim = new PhotonCameraSim(reefCamera, prop);
        visionSim = new VisionSystemSim("visionSim");
        visionSim.addAprilTags(fieldLayout);
        visionSim.addCamera(reefCameraSim, new Transform3d(0,0,0,new Rotation3d(0,0,0)));
        reefCameraSim.enableDrawWireframe(true);
        reefCameraSim.enableProcessedStream(true);
    }

    @Override
    public void periodic() {

        var pose = getRobotInTagSpace();
        SmartDashboard.putBoolean("Subsystem/posePresent", pose.isPresent());

        if (pose != null && pose.isPresent()) {
            reefDstPose = pose.get();
            reefTagDisp.set(reefDstPose);

            SmartDashboard.putBoolean("Subsystem/ALIGNED",
                (((Math.abs(reefDstPose.getX() - leftAlignmentX) < xTolerance)
                    && (Math.abs(reefDstPose.getY() - leftAlignmentY) < yTolerance)
                    && (Math.abs(reefDstPose.getRotation().getRadians() - thetaAlignment) < thetaTolerance))
                    || ((Math.abs(reefDstPose.getX() - rightAlignmentX) < xTolerance)
                        && (Math.abs(reefDstPose.getY() - rightAlignmentY) < yTolerance)
                        && (Math.abs(reefDstPose.getRotation().getRadians() - thetaAlignment) < thetaTolerance))));

            SmartDashboard.putBoolean("Subsystem/Vision_CAN_ALIGN",
                latestID != -1 && FieldPositions.isReefID(latestID)
                    && pose.get().getX() < maxAlignmentDistance);

        } else {
            SmartDashboard.putBoolean("Subsystem/Vision_CAN_ALIGN", false);
            SmartDashboard.putBoolean("Subsystem/ALIGNED", false);
        }

        var result = reefCamera.getLatestResult();
        if (result != null && result.hasTargets()) {
            latestID = result.getBestTarget().getFiducialId();
        } else {
            latestID = -1;
        }
    }

    public int getBestTag(boolean left) {
        List<PhotonTrackedTarget> targetsSeen = getTargetsSeen(left);
        if (targetsSeen != null) {
            int bestId = 0;
            double bestDistance = Double.MAX_VALUE;
            for (PhotonTrackedTarget t : targetsSeen) {
                if (isReefID(t.getFiducialId())) continue;
                double distance = Math.abs(t.getBestCameraToTarget().getY());
                if (distance < bestDistance) {
                    bestId = t.getFiducialId();
                    bestDistance = distance;
                }
            }
            return bestId;
        }
        // if some error with getting targets return -1
        return -1;
    }

    public List<PhotonTrackedTarget> getTargetsSeen(boolean left) {
        PhotonPipelineResult result = reefCamera.getLatestResult();
        if (result != null && result.hasTargets()) {
            return result.getTargets();
        }
        return null;
    }

    public Pose2d getEstimatedPose(boolean left) {
        PhotonPoseEstimator selectedPoseEstimator = poseEstimator;
        PhotonPipelineResult result = reefCamera.getLatestResult(); //deprecated, use getAllUnreadResults()
        if (result != null && result.hasTargets()) {
            Optional<EstimatedRobotPose> estimatedPose = selectedPoseEstimator.update(result);
            if (estimatedPose.isPresent()) {
                return estimatedPose.get().estimatedPose.toPose2d();
            }
        }
        return null;
    }

    public Pose2d getRobotToTagTransform(boolean left, int bestId) {
        // if no tag seen
        if (bestId == -1) return null;
        Pose2d tagPose = fieldLayout.getTagPose(bestId).get().toPose2d();
        Pose2d robotPose = getEstimatedPose(left);
        return robotPose.relativeTo(tagPose);
    }

    public Optional<Pose2d> getRobotInTagSpace() {
        PhotonPipelineResult result = reefCamera.getLatestResult(); //deprecated, use getAllUnreadResults()

        if (result != null && result.hasTargets()) {
            Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update(result);

            if (estimatedPoseOptional.isPresent()) {
                // this bestID stuff seems irrelevant, the lowest_ambiguity strategy of the pose estimator will do this for you. Just filter out non reef tags and let the estimator handle the rest
                EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
                double y = estimatedPose.estimatedPose.toPose2d().getRotation().getRadians();

                estimatedCaemraPose.set(estimatedPose.estimatedPose.toPose2d());
                int bestId = 0;
                double bestDistance = Double.MAX_VALUE;
                for (PhotonTrackedTarget t : result.getTargets()) {
                    if (isReefID(t.getFiducialId())) continue;
                    double distance = Math.abs(t.getBestCameraToTarget().getY());

                    if (distance < bestDistance) {
                        bestId = t.getFiducialId();
                        bestDistance = distance;
                    }
                }
                SmartDashboard.putNumber("Subsystem/Vision/BestReefId", bestId);
                Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(bestId);

                if (tagPoseOptional.isPresent()) {
                    Pose3d tagPose = tagPoseOptional.get();

                    Pose2d robotPose = estimatedPose.estimatedPose.toPose2d();
                    Pose2d tagPose2d = tagPose.toPose2d();
                    Pose2d robotInTagSpace = robotPose.relativeTo(tagPose2d);
                    lastCalculatedDist = Optional.of(robotInTagSpace);

                    return Optional.of(robotInTagSpace);
                }
            }
        }
        return lastCalculatedDist;
    }

    public static final int[] kReefIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

    public static boolean isReefID(int id) {
        for (int i : kReefIDs) {
            if (id == i) return true;
        }
        return false;
    }

    public void simulationPeriodic(Pose2d robotPose){
        visionSim.update(robotPose);
    }
}
