package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.Seconds;
import java.util.Map;
import java.util.HashMap;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.jni.WPIMathJNI;
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

public class Vision extends SubsystemBase {
    public static record VisionEstimate(Pose2d pose, double timestampSeconds, Vector<N3> stdDevs){}
    private final Drive drive;
    public static final double leftAlignmentX = .2435737274077523; // meters
    public static final double leftAlignmentY = 0.275;
    public static final double rightAlignmentX = .2435737274077523;
    public static final double rightAlignmentY = 0;
    public static final double thetaAlignment = -Math.PI / 2; // degrees
    public static double maxAlignmentDistance = 1.5;

    public static final double xTolerance = .05;
    public static final double yTolerance = .05;
    public static final double thetaTolerance = .05;

    public static final double robotCenterToEdge = Units.inchesToMeters(17.5);

    public static class FieldPositions {
        // Robot Positions when lining up to the tags, L means left branch, R means
        // right
        // Blue
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

        public static final Pose2d HP12 = new Pose2d(Units.inchesToMeters(33.51) + robotCenterToEdge*Math.cos(Math.toRadians(54)), Units.inchesToMeters(25.80) + robotCenterToEdge*Math.sin(Math.toRadians(54)), new Rotation2d(Math.toRadians(54)));
        public static final Pose2d HP13 = new Pose2d(Units.inchesToMeters(33.51) + robotCenterToEdge*Math.cos(Math.toRadians(54)), Units.inchesToMeters(291.20) - robotCenterToEdge*Math.sin(Math.toRadians(54)), new Rotation2d(Math.toRadians(306)));

        // Red
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

        public static final Pose2d HP1 = new Pose2d(Units.inchesToMeters(657.37) - robotCenterToEdge*Math.cos(Math.toRadians(54)), Units.inchesToMeters(25.80) + robotCenterToEdge*Math.sin(Math.toRadians(54)), new Rotation2d(Math.toRadians(126)));
        public static final Pose2d HP2 = new Pose2d(Units.inchesToMeters(657.37) - robotCenterToEdge*Math.cos(Math.toRadians(54)), Units.inchesToMeters(291.20) - robotCenterToEdge*Math.sin(Math.toRadians(54)), new Rotation2d(Math.toRadians(234)));

        public static final Pose2d HP6 = new Pose2d(
                (FieldPositions.R6.getX() + FieldPositions.L6.getX()) / 2,
                (FieldPositions.R6.getY() + FieldPositions.L6.getY()) / 2,
                new Rotation2d(
                        (FieldPositions.R6.getRotation().getRadians() + FieldPositions.L6.getRotation().getRadians())
                                / 2));

        public static final Pose2d HP8 = new Pose2d(
                (FieldPositions.R8.getX() + FieldPositions.L8.getX()) / 2,
                (FieldPositions.R8.getY() + FieldPositions.L8.getY()) / 2,
                new Rotation2d(
                        (FieldPositions.R8.getRotation().getRadians() + FieldPositions.L8.getRotation().getRadians())
                                / 2));

        public static final Pose2d HP17 = new Pose2d(
                (FieldPositions.R17.getX() + FieldPositions.L17.getX()) / 2,
                (FieldPositions.R17.getY() + FieldPositions.L17.getY()) / 2,
                new Rotation2d(
                        (FieldPositions.R17.getRotation().getRadians() + FieldPositions.L17.getRotation().getRadians())
                                / 2));

        public static final Pose2d HP19 = new Pose2d(
                (FieldPositions.R19.getX() + FieldPositions.L19.getX()) / 2,
                (FieldPositions.R19.getY() + FieldPositions.L19.getY()) / 2,
                new Rotation2d(
                        (FieldPositions.R19.getRotation().getRadians() + FieldPositions.L19.getRotation().getRadians())
                                / 2));

        public static final Pose2d BARGE_START = new Pose2d(7.449, 6.134, new Rotation2d(Math.toRadians(-30)));
        public static final Pose2d PROCESSOR_START = new Pose2d(7.449, 6.134, new Rotation2d(Math.toRadians(-30)));
        public static final List<Pose2d> kLeftReefPoses = Arrays.asList(
                L17, L18, L19, L20, L21, L22, L6, L7, L8, L9, L10, L11);
        public static final List<Pose2d> kRightReefPoses = Arrays.asList(
                R17, R18, R19, R20, R21, R22, R6, R7, R8, R9, R10, R11);

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4);
        public static final Translation2d RED_REEF_CENTER = new Translation2d(13, 4);

        public static final int[] kReefIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

        public static final int[] hpIDs = { 6, 8, 17, 19 };

        public static boolean isReefID(int id) {
            for (int i : kReefIDs) {
                if (id == i)
                    return true;
            }
            return false;
        }

        public static boolean isHPID(int id) {
            for (int i : hpIDs) {
                if (id == i)
                    return true;
            }
            return false;
        }

        public static final Pose2d BLUE_LEFT_CORAL_STATION_PICKUP = new Pose2d(new Translation2d(1.2, 7),
                Rotation2d.fromDegrees(120));
        public static final Pose2d BLUE_CLIMB_AREA = new Pose2d(new Translation2d(7.638, 6.174),
                Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_PROCESSOR = new Pose2d(new Translation2d(6.332, .52),
                Rotation2d.fromDegrees(-90));
    }

    // Mapping from left deposit poses to their corresponding AprilTag IDs
    private static final Map<Pose2d, Integer> leftDepositMapping = new HashMap<>() {
        {
            put(FieldPositions.L17, 17);
            put(FieldPositions.L18, 18);
            put(FieldPositions.L19, 19);
            put(FieldPositions.L20, 20);
            put(FieldPositions.L21, 21);
            put(FieldPositions.L22, 22);
            put(FieldPositions.L6, 6);
            put(FieldPositions.L7, 7);
            put(FieldPositions.L8, 8);
            put(FieldPositions.L9, 9);
            put(FieldPositions.L10, 10);
            put(FieldPositions.L11, 11);
        }
    };

    // Mapping from right deposit poses to their corresponding AprilTag IDs
    private static final Map<Pose2d, Integer> rightDepositMapping = new HashMap<>() {
        {
            put(FieldPositions.R17, 17);
            put(FieldPositions.R18, 18);
            put(FieldPositions.R19, 19);
            put(FieldPositions.R20, 20);
            put(FieldPositions.R21, 21);
            put(FieldPositions.R22, 22);
            put(FieldPositions.R6, 6);
            put(FieldPositions.R7, 7);
            put(FieldPositions.R8, 8);
            put(FieldPositions.R9, 9);
            put(FieldPositions.R10, 10);
            put(FieldPositions.R11, 11);
        }
    };

    public static AprilTagFieldLayout fieldLayout;

    private PhotonCamera rightCamera;
    private PhotonCamera leftCamera;
    private Optional<Pose2d> lastCalculatedDist;
    private PhotonPoseEstimator rightPoseEstimator;
    private PhotonPoseEstimator leftPoseEstimator;
    private int latestID;
    private Pose2d reefDstPose;
    // private static Transform3d rightCameraTransform = new
    // Transform3d(.177,-.299,.223+0.036073,new
    // Rotation3d(0,11.508393*0.01745329,31.474949*0.01745329));
    // private static Transform3d leftCameraTransform = new Transform3d(.177, .299,
    // .223 + 0.036073, new Rotation3d(0, 11.508393 * 0.01745329, -31.474949 *
    // 0.01745329));

    private static Transform3d rightCameraTransform = new Transform3d(.159, -.272, 0.284988,
            new Rotation3d(0, 0, Math.toRadians(15)));
    private static Transform3d leftCameraTransform = new Transform3d(.159, .272, 0.284988,
            new Rotation3d(0, 0, -Math.toRadians(16)));
    private PhotonCameraSim reefCameraSim;
    private PhotonCameraSim leftCameraSim;
    private VisionSystemSim visionSim;
    private SimCameraProperties prop;
    StructPublisher<Pose2d> reefTagDisp = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Subsystem/Vision/RobotToTag", Pose2d.struct).publish();
    StructPublisher<Pose2d> estimatedCaemraPose = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Subsystem/Vision/estimatedCameraPose", Pose2d.struct).publish();

    public Vision(Drive drive) {
        this.drive = drive;

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        rightCamera = new PhotonCamera("rightCamera");
        leftCamera = new PhotonCamera("leftCamera");
        rightPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                rightCameraTransform);
        leftPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                leftCameraTransform);
        lastCalculatedDist = Optional.empty();

        prop = new SimCameraProperties();
        prop.setFPS(50);
        prop.setAvgLatencyMs(50);
        prop.setCalibError(.35, .10);
        prop.setLatencyStdDevMs(15);
        prop.setCalibration(1600, 1200, Rotation2d.fromDegrees(75));
        reefCameraSim = new PhotonCameraSim(rightCamera, prop);
        leftCameraSim = new PhotonCameraSim(leftCamera, prop);
        visionSim = new VisionSystemSim("visionSim");
        visionSim.addAprilTags(fieldLayout);
        visionSim.addCamera(reefCameraSim, rightCameraTransform);
        visionSim.addCamera(leftCameraSim, leftCameraTransform);
        reefCameraSim.enableDrawWireframe(true);
        reefCameraSim.enableProcessedStream(true);
        leftCameraSim.enableDrawWireframe(true);
        leftCameraSim.enableProcessedStream(true);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Left Tags Seen", leftCamera.getLatestResult().getTargets().size());
        SmartDashboard.putNumber("Right Tags Seen", rightCamera.getLatestResult().getTargets().size());

        var rightPose = getRobotInTagSpace(true);
        SmartDashboard.putBoolean("Subsystem/posePresent", rightPose.isPresent());
        SmartDashboard.putString("tagPose", rightPose.toString());

        var leftPose = getRobotInTagSpace(false); // Use left camera for left-side alignment
        SmartDashboard.putBoolean("Subsystem/leftPosePresent", leftPose.isPresent());
        SmartDashboard.putString("leftTagPose", leftPose.toString());

        if (rightPose != null && rightPose.isPresent()) {
            reefDstPose = rightPose.get();
            reefTagDisp.set(reefDstPose);

            SmartDashboard.putBoolean("Subsystem/ALIGNED",
                    (((Math.abs(reefDstPose.getX() - leftAlignmentX) < xTolerance)
                            && (Math.abs(reefDstPose.getY() - leftAlignmentY) < yTolerance)
                            && (Math.abs(reefDstPose.getRotation().getRadians() - thetaAlignment) < thetaTolerance))
                            || ((Math.abs(reefDstPose.getX() - rightAlignmentX) < xTolerance)
                                    && (Math.abs(reefDstPose.getY() - rightAlignmentY) < yTolerance)
                                    && (Math.abs(reefDstPose.getRotation().getRadians()
                                            - thetaAlignment) < thetaTolerance))));

            SmartDashboard.putBoolean("Subsystem/Vision_CAN_ALIGN",
                    latestID != -1 && FieldPositions.isReefID(latestID)
                            && rightPose.get().getX() < maxAlignmentDistance);

        } else {
            SmartDashboard.putBoolean("Subsystem/Vision_CAN_ALIGN", false);
            SmartDashboard.putBoolean("Subsystem/ALIGNED", false);
        }

        var result = rightCamera.getLatestResult();
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
                if (!isReefID(t.getFiducialId()))
                    continue;
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
        PhotonPipelineResult result = left ? leftCamera.getLatestResult() : rightCamera.getLatestResult();
        if (result != null && result.hasTargets()) {
            return result.getTargets();
        }
        return null;
    }

    public Pose2d getEstimatedPose(boolean left) {
        PhotonPipelineResult result = left ? leftCamera.getLatestResult() : rightCamera.getLatestResult();
        PhotonPoseEstimator selectedPoseEstimator = left ? leftPoseEstimator : rightPoseEstimator;

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
        if (bestId == -1)
            return null;
        Pose2d tagPose = fieldLayout.getTagPose(bestId).get().toPose2d();
        Pose2d robotPose = getEstimatedPose(left);
        return robotPose.relativeTo(tagPose);
    }

    public Optional<Pose2d> getRobotInTagSpace(boolean left, boolean hp) {
        // Get the latest vision result
        PhotonPipelineResult result = left ? leftCamera.getLatestResult() : rightCamera.getLatestResult();
        // PhotonPoseEstimator selectedPoseEstimator = left ? leftPoseEstimator :
        // rightPoseEstimator;

        if (result == null || !result.hasTargets()) {
            return lastCalculatedDist;
        }

        // Use the photon pose estimator to update the global robot pose based on the
        // vision result.
        // Optional<EstimatedRobotPose> globalPoseOpt =
        // selectedPoseEstimator.update(result);
        // if (!globalPoseOpt.isPresent()) {
        // return lastCalculatedDist;
        // }
        Pose2d globalPose = drive.getPose();

        // Choose the list of deposit poses (for example, the left positions).
        // If you later need to toggle between left/right, you might pass a flag or
        // determine it some other way.

        Pose2d nearestDepositPose;
        if (hp) {
            nearestDepositPose = fieldLayout.getTags().stream().filter(tagPose -> isHPID(tagPose.ID))
                    .map(tagPose -> tagPose.pose.toPose2d())
                    .min((p1, p2) -> Double.compare(globalPose.getTranslation().getDistance(p1.getTranslation()),
                            globalPose.getTranslation().getDistance(p2.getTranslation())))
                    .orElse(null);
        } else {
            nearestDepositPose = fieldLayout.getTags().stream().filter(tagPose -> isReefID(tagPose.ID))
                    .map(tagPose -> tagPose.pose.toPose2d())
                    .min((p1, p2) -> Double.compare(globalPose.getTranslation().getDistance(p1.getTranslation()),
                            globalPose.getTranslation().getDistance(p2.getTranslation())))
                    .orElse(null);
        }
        if (nearestDepositPose == null) {
            return lastCalculatedDist;
        }

        Pose2d relativePose = globalPose.relativeTo(nearestDepositPose);
        SmartDashboard.putString("Relative Pose", relativePose.toString());

        lastCalculatedDist = Optional.of(relativePose);

        return Optional.of(relativePose);

    }

    public Optional<Pose2d> getRobotInTagSpace(boolean left) {
        return getRobotInTagSpace(left, false);
    }

    /////////////////////////////////////// Global Pose
    /////////////////////////////////////// //////////////////////////////////////////////

    private static boolean isSane(EstimatedRobotPose est,
            PhotonPipelineResult src,
            Pose2d reference) {
        Pose2d p = est.estimatedPose.toPose2d();
        double dist = p.getTranslation().getDistance(reference.getTranslation());
        double amb = (src != null && src.hasTargets() && src.getBestTarget() != null)
                ? src.getBestTarget().getPoseAmbiguity()
                : -1.0;

        // Reject sketchy single-tag solutions that jump far from reference with high
        // ambiguity.
        if (amb > 0.25 && dist > 1) {
            return false;
        }
        return true;
    }
    private Vector<N3> calculateStdValues(EstimatedRobotPose estimate, Pose2d currentPose){
        double xValue = 0.1;
        double yValue = 0.1;
        double rotValue = 0.1;
        double distance = estimate.estimatedPose.toPose2d().getTranslation().getDistance(currentPose.getTranslation());
        int tags = estimate.targetsUsed.size();
        // double averageAmbiguity = estimate.targetsUsed.stream().mapToDouble(x -> x.poseAmbiguity).average().getAsDouble();
        // double maxTargetAngle = estimate.targetsUsed.stream().mapToDouble(x -> x.bestCameraToTarget.getRotation().getAngle()).max().getAsDouble();
        // System.out.println(maxTargetAngle);

        if (distance > 1){
            double distanceSquared = distance * distance;
            xValue*=distanceSquared;
            yValue*=distanceSquared;
            rotValue*=distanceSquared;

            if (tags < 2) {
                xValue*=200;
                yValue*=200;
                rotValue*=200;
            }
        }

        return VecBuilder.fill(xValue, yValue, rotValue);

    }

    public List<EstimatedRobotPose> getAllVisionEstimates() {
        List<EstimatedRobotPose> results = new ArrayList<>();

        leftCamera.getAllUnreadResults().forEach(x -> {
            leftPoseEstimator.update(x).ifPresent(results::add);
        });

        rightCamera.getAllUnreadResults().forEach(x -> {
            rightPoseEstimator.update(x).ifPresent(results::add);
        });

        return results;
    }

    public Optional<VisionEstimate> getEstimatedGlobalPoseRight(Pose2d prevEstimatedRobotPose) {
        // 1) Grab latest results
        PhotonPipelineResult rightResult = rightCamera.getLatestResult();
        // if (rightResult.getTargets().size() < 2){
        // return Optional.empty();
        // }

        Optional<EstimatedRobotPose> rightEst = Optional.empty();

        if (rightResult != null && rightResult.hasTargets()) {
            rightEst = rightPoseEstimator.update(rightResult);
        }

        // 2) Sanity check
        if (rightEst.isPresent() && !isSane(rightEst.get(), rightResult, prevEstimatedRobotPose)) {
            rightEst = Optional.empty();
        }

        // 3) If the estimate is present, return it
        if (rightEst.isPresent()) {
            return Optional.of(new VisionEstimate(rightEst.get().estimatedPose.toPose2d(), rightEst.get().timestampSeconds, calculateStdValues(rightEst.get(), prevEstimatedRobotPose)));
        }

        return Optional.empty();
    }

    public Optional<VisionEstimate> getEstimatedGlobalPoseLeft(Pose2d prevEstimatedRobotPose) {
        // 1) Grab latest results
        PhotonPipelineResult leftResult = leftCamera.getLatestResult();
        // if (leftResult.getTargets().size() < 2){
        // return Optional.empty();
        // }

        Optional<EstimatedRobotPose> leftEst = Optional.empty();

        if (leftResult != null && leftResult.hasTargets()) {
            leftEst = leftPoseEstimator.update(leftResult);
        }

        // 2) Sanity check
        if (leftEst.isPresent() && !isSane(leftEst.get(), leftResult, prevEstimatedRobotPose)) {
            leftEst = Optional.empty();
        }

        // 3) If the estimate is present, return it
        if (leftEst.isPresent()) {
            return Optional.of(new VisionEstimate(leftEst.get().estimatedPose.toPose2d(), leftEst.get().timestampSeconds, calculateStdValues(leftEst.get(), prevEstimatedRobotPose)));
        }

        return Optional.empty();
    }

    // public static final int[] kReefIDs = {6};

    public static final int[] kReefIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public static boolean isReefID(int id) {
        for (int i : kReefIDs) {
            if (id == i)
                return true;
        }
        return false;
    }

    public static final int[] hpIDs = { 6, 8, 17, 19 };

    public static boolean isHPID(int id) {
        for (int i : FieldPositions.hpIDs) {
            if (id == i)
                return true;
        }
        return false;
    }

    public void simulationPeriodic(Pose2d robotPose) {
        visionSim.update(robotPose);
    }
}
