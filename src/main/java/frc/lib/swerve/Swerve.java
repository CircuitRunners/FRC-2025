package frc.lib.swerve;


//ctre imports
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;

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
import edu.wpi.first.wpilibj2.command.Command;

public class Swerve extends LegacySwerveDrivetrain {
    //private SwerveRequest.ApplyChassisSpeeds autorequest = new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(LegacySwerveDrivetrainConstants drivetrainConstants, LegacySwerveModuleConstants[] moduleConstants){
        super(drivetrainConstants, moduleConstants);
    }

    public ChassisSpeeds getChassisSpeeds(){
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }


    public SwerveModuleState[] getModuleStates(){
        return getState().ModuleStates != null? getState().ModuleStates : new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};
    }

    public Pose2d getPose2d(){
        return getState().Pose != null ? getState().Pose : new Pose2d();
    }

    public SwerveDriveKinematics getDriveKinematics(){
        return m_kinematics;
    }

    public void resetPose(Pose2d pose){
        try{
            m_stateLock.writeLock().lock();
            m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
        } finally{

        }
    }

    public Command zeroGyroCommand(){
        return Commands.runOnce(() -> m_pigeon2.reset());
    }

    public double getPitch(){
        return m_pigeon2.getPitch().getValueAsDouble();
    }

    public void targetAngleDrive(Translation2d targetAngle, double forward, double strafe){
        setControl((new LegacySwerveRequest.FieldCentricFacingAngle().withCenterOfRotation(targetAngle).withVelocityX(forward).withVelocityY(strafe)));
    }

    public void targetAngleDrive(Rotation2d targetAngle, double forward, double strafe){
        setControl((new LegacySwerveRequest.FieldCentricFacingAngle().withTargetDirection(targetAngle).withVelocityX(forward).withVelocityY(strafe)));
    }

    public void addVisionMeasurement(Pose2d pose2d, double timestamp){
        m_odometry.addVisionMeasurement(pose2d, timestamp);
    }

    public void setPigeonOffset(){
        m_pigeon2.getConfigurator().apply((new MountPoseConfigs()).withMountPosePitch(47.109375).withMountPoseRoll(-88.242188).withMountPoseYaw(-83.410950));
    }

}