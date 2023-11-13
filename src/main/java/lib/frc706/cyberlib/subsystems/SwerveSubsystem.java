package lib.frc706.cyberlib.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class SwerveSubsystem extends SubsystemBase {
    private final double MAX_VELOCITY_METERS_PER_SECOND;
    AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
    private final SwerveModule[] swerveModules;

    private static SwerveDrivePoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();

    private final SwerveDriveKinematics kDriveKinematics;

    private PhotonCameraWrapper[] cameras;
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    public SwerveSubsystem(SwerveModule.ModuleType moduleType, int wheelBase, int[] driveMotorPorts, int[] turningMotorPorts, int[] absoluteEncoderPorts, double[] absoluteEncoderOffsets, boolean driveMotorsInverted[], boolean[] turningMotorsInverted, boolean[] absoluteEncodersInverted, HolonomicPathFollowerConfig pathFollowerConfig, PhotonCameraWrapper... cameras) {
        frontLeft = new SwerveModule(moduleType, SwerveModule.ModulePosition.FL, driveMotorPorts[0], turningMotorPorts[0], absoluteEncoderPorts[0], absoluteEncoderOffsets[0], driveMotorsInverted[0], turningMotorsInverted[0], absoluteEncodersInverted[0]);
        frontRight = new SwerveModule(moduleType, SwerveModule.ModulePosition.FR, driveMotorPorts[1], turningMotorPorts[1], absoluteEncoderPorts[1], absoluteEncoderOffsets[1], driveMotorsInverted[1], turningMotorsInverted[1], absoluteEncodersInverted[1]);
        backLeft = new SwerveModule(moduleType, SwerveModule.ModulePosition.BL, driveMotorPorts[2], turningMotorPorts[2], absoluteEncoderPorts[2], absoluteEncoderOffsets[2], driveMotorsInverted[2], turningMotorsInverted[2], absoluteEncodersInverted[2]);
        backRight = new SwerveModule(moduleType, SwerveModule.ModulePosition.BR, driveMotorPorts[3], turningMotorPorts[3], absoluteEncoderPorts[3], absoluteEncoderOffsets[3], driveMotorsInverted[3], turningMotorsInverted[3], absoluteEncodersInverted[3]);
        swerveModules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };
        MAX_VELOCITY_METERS_PER_SECOND = frontLeft.MAX_VELOCITY_METERS_PER_SECOND;
        updatePositions();
        this.cameras = cameras;
        kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, wheelBase / 2), // FL,FR,BL,BR
                new Translation2d(wheelBase / 2, -wheelBase / 2),
                new Translation2d(-wheelBase / 2, wheelBase / 2),
                new Translation2d(-wheelBase / 2, -wheelBase / 2));
        poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, getRotation2d(), modulePosition, new Pose2d());
        SmartDashboard.putData("Field", m_field);
        recenter();
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            pathFollowerConfig,
            this // Reference to this subsystem to set requirements
        );
    }

    public SwerveDriveKinematics getKinematics() {
        return kDriveKinematics;
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), modulePosition, pose);
    }

    public void recenter() {
        resetOdometry(new Pose2d());
        zeroHeading();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * move the robot
     * 
     * @param xSpeed forwards speed, positive is away from our alliance wall
     * @param ySpeed sideways speed, positive is left
     * @param rot rotation speed, positive is counterclockwise
     * @param fieldRelative whether the xSpeed and ySpeed are relative to the field
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setModuleStates(moduleStates);
    }

    /**
     * set the module states
     * @param speeds robot-relative chassis speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    @Override
    public void periodic() {
        updatePositions();
        poseEstimator.update(getRotation2d(), modulePosition);
        for (PhotonCameraWrapper camera : cameras) {
            Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(getPose());
            if (result.isPresent()) {
                poseEstimator.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
            }
        }
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    private void updatePositions() {
        for(int i = 0; i < swerveModules.length; i++) {
            modulePosition[i] = swerveModules[i].getPosition();
            SmartDashboard.putString("Swerve[" + swerveModules[i] + "] state", swerveModules[i].getPosition().toString());
        }
    }

    public void stopModules() {
        for(SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
        for(int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    public void toggleModulesLocked() {
        for(SwerveModule module : swerveModules) {
            module.toggleLocked();
        }
    }

    public Command lockModulesCommand() {
        return new RepeatCommand(new InstantCommand(() -> toggleModulesLocked())).withTimeout(1).andThen(this::stopModules);
    }
}