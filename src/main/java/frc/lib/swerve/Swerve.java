package frc.lib.swerve;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
//ctre imports
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

//edu.wpi imports
//math.geometry

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

//math.kinematics

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//command imports

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class Swerve extends TunerSwerveDrivetrain {

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        // if (Utils.isSimulation()) {
        //     startSimThread();
        // }
        // configureAutoBuilder();
    }

    public SwerveModuleState[] getModuleStates(){
        return getState().ModuleStates != null? getState().ModuleStates : new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};
    }

    public Pose2d getPose2d(){
        return getState().Pose != null ? getState().Pose : new Pose2d();
    }


}