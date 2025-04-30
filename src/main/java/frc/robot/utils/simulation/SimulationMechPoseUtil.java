package frc.robot.utils.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

public class SimulationMechPoseUtil {
    private static Supplier<Pose3d> pose1Supplier;
    private static Supplier<Pose3d> pose2Supplier;
    private static Supplier<Rotation3d> rot3Supplier;
    /**
     * Creates a new SimulationMechPoseUtil with three Pose3d suppliers, two from the elevator and one from the claw
     *
     * @param pose1Supplier Elevator Stage 1 Pose3d Supplier
     * @param pose2Supplier Elevator Stage 2 Pose3d Supplier
     * @param rot3Supplier Arm Rotation3d Supplier
     * @see Elevator
     */
    public SimulationMechPoseUtil(
            Supplier<Pose3d> pose1Supplier, Supplier<Pose3d> pose2Supplier, Supplier<Rotation3d> rot3Supplier) {
        SimulationMechPoseUtil.pose1Supplier = pose1Supplier;
        SimulationMechPoseUtil.pose2Supplier = pose2Supplier;
        SimulationMechPoseUtil.rot3Supplier = rot3Supplier;
    }

    /**
     * Static method to get the pose of the elevator stage 1, directly from the elevator PID controller position
     *
     * @return Simulated pose of the elevator stage 1
     * @see Elevator
     */
    public static Pose3d getPose1() {
        return pose1Supplier.get();
    }

    /**
     * Static method to get the pose of the elevator stage 2, which is simply 2x the height of stage 1, due to the
     * cascade rigging of the elevator
     *
     * @return Simulated pose of the elevator stage 2
     * @see Elevator
     */
    public static Pose3d getPose2() {
        return pose2Supplier.get();
    }

    /**
     * Static method to get the pose of the arm. Uses the pose of the 2nd elevator stage as a reference, applies a
     * transform3d to move to the pivot axis of the claw, and applies the rotation of the arm from the rot3Supplier
     *
     * @return Simulated pose of the arm
     * @see Claw
     * @see Elevator
     */
    public static Pose3d getPose3() {
        return pose2Supplier.get().plus(new Transform3d(0.247397, 0, 0.504439, rot3Supplier.get()));
    }

    /**
     * Static method to get the poses of all three components. called in RobotPeriodic of Robot.java and logged using
     * DogLog so it can be viewed in AdvantageScope
     *
     * @return Array of Pose3d objects, in the order of elevator stage 1, elevator stage 2, and arm
     */
    public static Pose3d[] getPoses() {
        return new Pose3d[] {getPose1(), getPose2(), getPose3()};
    }

    /**
     * Static method to get the zeroed poses of all three components, used as reference for simulation setup
     *
     * @return
     */
    public static Pose3d[] zeroedPose3ds() {
        return new Pose3d[] {
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))
        };
    }
}
