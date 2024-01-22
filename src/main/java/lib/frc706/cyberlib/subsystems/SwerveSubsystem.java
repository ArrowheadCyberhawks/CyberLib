package lib.frc706.cyberlib.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private final ShuffleboardLayout layout;

    public SwerveSubsystem(SwerveModule moduleFL, SwerveModule moduleFR, SwerveModule moduleBL, SwerveModule moduleBR,
            int wheelBase, HolonomicPathFollowerConfig pathFollowerConfig, PhotonCameraWrapper... cameras) {
        layout = Shuffleboard.getTab("SwerveDrive").getLayout("SwerveDrive", BuiltInLayouts.kGrid);
        frontLeft = moduleFL;
        frontRight = moduleFR;
        backLeft = moduleBL;
        backRight = moduleBR;
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
        layout.add("Field", m_field);
        layout.addDouble("Robot Heading", () -> getHeading());
        recenter();
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                pathFollowerConfig,
                this::getFlipPath,
                this // Reference to this subsystem to set requirements
        );
        setName("SwerveSubsystem");
    }

    /**
     * Creates a new SwerveSubsystem with the given parameters.
     * <p>
     * All arrays should be in the order FL, FR, BL, BR.
     * </p>
     */
    public SwerveSubsystem(ModuleType moduleType, double wheelBase, int[] driveMotorIDs, int[] turningMotorIDs,
            int[] absoluteEncoderPorts, double[] absoluteEncoderOffsets, boolean[] driveMotorsInverted,
            boolean[] turningMotorsInverted, boolean[] absoluteEncodersInverted,
            HolonomicPathFollowerConfig pathFollowerConfig, PhotonCameraWrapper... cameras) {
        layout = Shuffleboard.getTab("SwerveDrive").getLayout("SwerveDrive", BuiltInLayouts.kGrid);
        frontLeft = new SwerveModule(moduleType, SwerveModule.ModulePosition.FL, driveMotorIDs[0], turningMotorIDs[0],
                absoluteEncoderPorts[0], absoluteEncoderOffsets[0], driveMotorsInverted[0], turningMotorsInverted[0],
                absoluteEncodersInverted[0]);
        frontRight = new SwerveModule(moduleType, SwerveModule.ModulePosition.FR, driveMotorIDs[1], turningMotorIDs[1],
                absoluteEncoderPorts[1], absoluteEncoderOffsets[1], driveMotorsInverted[1], turningMotorsInverted[1],
                absoluteEncodersInverted[1]);
        backLeft = new SwerveModule(moduleType, SwerveModule.ModulePosition.BL, driveMotorIDs[2], turningMotorIDs[2],
                absoluteEncoderPorts[2], absoluteEncoderOffsets[2], driveMotorsInverted[2], turningMotorsInverted[2],
                absoluteEncodersInverted[2]);
        backRight = new SwerveModule(moduleType, SwerveModule.ModulePosition.BR, driveMotorIDs[3], turningMotorIDs[3],
                absoluteEncoderPorts[3], absoluteEncoderOffsets[3], driveMotorsInverted[3], turningMotorsInverted[3],
                absoluteEncodersInverted[3]);
        swerveModules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };
        MAX_VELOCITY_METERS_PER_SECOND = moduleType.getMaxVelocity();
        updatePositions();
        this.cameras = cameras;
        kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, wheelBase / 2), // FL,FR,BL,BR
                new Translation2d(wheelBase / 2, -wheelBase / 2),
                new Translation2d(-wheelBase / 2, wheelBase / 2),
                new Translation2d(-wheelBase / 2, -wheelBase / 2));
        poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, getRotation2d(), modulePosition, new Pose2d());
        layout.add("Field", m_field);
        recenter();
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                pathFollowerConfig,
                this::getFlipPath,
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

    public boolean getFlipPath() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) { //check if alliance is red or blue
            if (ally.get() == Alliance.Red) {
                return true;
            }
            if (ally.get() == Alliance.Blue) {
                return false;
            }
        }
        return false; //if we don't know then just give up and say no
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), modulePosition, pose);
    }

    /**
     * Recenter the robot's position on the field to (0,0) facing forwards.
     */
    public void recenter() {
        resetOdometry(new Pose2d());
        zeroHeading();
    }

    /**
     * Get the robot's current pose on the field.
     * @return the robot's current pose
     */
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
     * @param xSpeed        forwards speed, positive is away from our alliance wall
     * @param ySpeed        sideways speed, positive is left
     * @param rot           rotation speed, positive is counterclockwise
     * @param fieldRelative whether the xSpeed and ySpeed are relative to the field
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setModuleStates(moduleStates);
    }

    /**
     * set the module states
     * 
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
                poseEstimator.addVisionMeasurement(result.get().estimatedPose.toPose2d(),
                        result.get().timestampSeconds);
            }
        }
        m_field.setRobotPose(getPose());
    }

    private void updatePositions() {
        for (int i = 0; i < swerveModules.length; i++) {
            modulePosition[i] = swerveModules[i].getPosition();
        }
    }

    public void stopModules() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    public void toggleModulesLocked() {
        for (SwerveModule module : swerveModules) {
            module.toggleLocked();
        }
    }

    public Command lockModulesCommand() {
        return new InstantCommand(() -> toggleModulesLocked())
                .andThen(() -> this.setModuleStates(getModuleStates()))
                .withTimeout(1.0)
                .andThen(this::stopModules);
    }
}