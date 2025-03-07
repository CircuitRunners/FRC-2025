package frc.lib.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.swerve.SwerveConfig;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;


public class PathPlannerUtil {
    private static final StructSubscriber<Pose2d> kTargetPoseSub = NetworkTableInstance.getDefault().getStructTopic("targetPose", Pose2d.struct).subscribe(new Pose2d(), new PubSubOption[] {PubSubOption.periodic(0.2)});

    public static void configure(Drive drive, boolean simulation){
        // HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
        //     SwerveConstants.translationalPID, 
        //     SwerveConstants.rotationalPID, 
        //     TunerConstants.kSpeedAt12VoltsMps, 
        //     SwerveConstants.driveBaseRadiusMeter, 
        //     new ReplanningConfig(true, true)
        // );
        PathFollowingController controller = simulation ? new PPHolonomicDriveController(
            new PIDConstants(5, 0, 0),
            new PIDConstants(5, 0, 0)) : new PPHolonomicDriveController(SwerveConfig.translationPID,SwerveConfig.rotationPID); 
        // ModuleConfig moduleConfig = new ModuleConfig(TunerConstants.kWheelRadius, TunerConstants.kSpeedAt12Volts, SwerveConstants.wheelCOF, DCMotor.getKrakenX60(1), TunerConstants.kSlipCurrent, 1); 
        // RobotConfig robotConfig = new RobotConfig(SwerveConstants.robotMass, SwerveConstants.MOI,moduleConfig, Meters.convertFrom(28, Inches));

        RobotConfig robotConfig = null;
        try{
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }


        var somethign = DriverStation.getAlliance();
            if (somethign.isPresent()) {
                System.err.println(somethign.get() == DriverStation.Alliance.Red);
        }

        AutoBuilder.configure(
            drive::getPose,
            drive::resetPose,
            drive::getChassisSpeeds,
            drive::driveRobotCentric,
            controller,
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            drive
        );

    }

    public static Command getAutoCommand(String name){
        try {
            return AutoBuilder.buildAuto(name);
        } catch (Exception e) {
            // TODO: handle exception
            DriverStation.reportError("An error loaded while loading path planner auto", e.getStackTrace());
            return Commands.none();
        }
    }

    public static List<String> getAutos(){
        var path = Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner");
        try(Stream<Path> stream = Files.walk(path)){
            return stream.filter(x -> getFileExtension(x).equals(".auto")).map(x -> getFileStem(x)) .toList();
        }catch(IOException e){
            System.err.println("An error occurred while loading autos");
            e.printStackTrace();
            return Collections.emptyList();
        }
    }

    private static String getFileExtension(Path path){
        try{
            String name = path.getFileName().toString();
            return name.substring(name.lastIndexOf("."));
        } catch(Exception e){
            return "";
        }
    }

    private static String getFileStem(Path path){
        try{
            String name = path.getFileName().toString();
            return name.substring(0,name.lastIndexOf("."));
        }catch(Exception e){
            return "";
        }
    }

    public static Pose2d getCurrentTargetPose(){
        return kTargetPoseSub.get();
    }

}
    
