package lib.frc706.cyberlib.subsystems;

import static edu.wpi.first.units.Units.*;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveDrive swerveDrive;
    private final ShuffleboardLayout layout;
    private final PhotonCameraWrapper[] cameras;
    private RobotConfig config;
    private PIDConstants translationConstants, thetaConstants; //this is only here because YAGSL won't give us the one from the JSON file
    private Alert poseAlert = new Alert(getName(), "No robot pose!", AlertType.kWarning);
    

    private static double headingOffset = 0; // Difference between actual robot angle and angle of forward driving

    /**
     * Constructs a new SwerveSubsystem.
     *
     * @param configDir The directory containing the configuration files.
     * @param maxVel The maximum chassis velocity of the swerve drive in m/s.
     * @param translationConstants The PID constants for translation control.
     * @param cameras The PhotonCameraWrapper objects for vision processing.
     * 
     * @throws RuntimeException if the swerve drive creation fails.
     */
    public SwerveSubsystem(File configDir, double maxVel, PIDConstants translationConstants, PIDConstants thetaConstants, PhotonCameraWrapper... cameras) {
        layout = Shuffleboard.getTab("SwerveDrive").getLayout("SwerveDrive", BuiltInLayouts.kGrid);
        //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(configDir).createSwerveDrive(maxVel);
        } catch (IOException e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }
        this.cameras = cameras;
        this.translationConstants = translationConstants;
        this.thetaConstants = thetaConstants;
        layout.addDouble("Robot Heading", () -> getHeading());
        recenter();
        try {
            this.config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace(); 
            this.config = new RobotConfig(Kilograms.of(swerveDrive.swerveDriveConfiguration.physicalCharacteristics.robotMassKg),
               MomentOfInertia.ofRelativeUnits(swerveDrive.swerveDriveConfiguration.physicalCharacteristics.steerRotationalInertia, KilogramSquareMeters),
                new ModuleConfig(Meters.of(swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters()),
                MetersPerSecond.of(swerveDrive.getMaximumModuleDriveVelocity()),
                swerveDrive.swerveDriveConfiguration.modules[0].configuration.physicalCharacteristics.wheelGripCoefficientOfFriction,
                swerveDrive.swerveDriveConfiguration.getDriveMotorSim(),
                Amps.of(swerveDrive.swerveDriveConfiguration.modules[0].configuration.physicalCharacteristics.driveMotorCurrentLimit),
                1),
                swerveDrive.swerveDriveConfiguration.moduleLocationsMeters
            );
        }
        
        swerveDrive.setHeadingCorrection(false);
        setName("SwerveSubsystem");
        configurePathPlanner();
        swerveDrive.zeroGyro();
        ((com.studica.frc.AHRS) swerveDrive.getGyro().getIMU()).zeroYaw();
    }

    @Override
    public void periodic() {
        // !! MONKEY CODE !!
        for (PhotonCameraWrapper camera : cameras) {
            Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(getPose());
            if (result.isPresent() && Math.abs(getRobotRelativeSpeeds().omegaRadiansPerSecond) < 4 * Math.PI) { 
                double minDistance = Double.MAX_VALUE;
                for (var target : result.get().targetsUsed) {
                    double distance = target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                }
                minDistance *= 3;
                swerveDrive.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds,
                VecBuilder.fill(minDistance, minDistance, minDistance));
            }
        }
        // !! MONKEY CODE !!

        // for (PhotonCameraWrapper camera : cameras) {
        //     PhotonPipelineResult result = camera.photonCamera.getLatestResult();
        //             camera.photonPoseEstimator.setLastPose(getPose());
        //     Optional<EstimatedRobotPose> estimatedPose = camera.getEstimatedGlobalPose(getPose());
        //     if (estimatedPose.isPresent() && Math.abs(getRobotRelativeSpeeds().omegaRadiansPerSecond) < 4 * Math.PI) {
        //         boolean rejectUpdate = false;
        //         double maxAmbiguity = 0;
        //         for (var target : estimatedPose.get().targetsUsed) {
        //             // if(Math.abs(target.get()) < 20) {
        //             //     rejectUpdate = true;
        //             // }
        //             // System.out.println("Skew: " + target.wait(0, 0););
        //             double ambiguity = target.getPoseAmbiguity();
        //             if(ambiguity > 0.1)
        //                 rejectUpdate = true;    
        //             else if (ambiguity > maxAmbiguity)
        //                 maxAmbiguity = ambiguity;
                    
        //             // we hate the barge tags
        //             if (target.getFiducialId() == 4 || target.getFiducialId() == 5 || target.getFiducialId() == 14 || target.getFiducialId() == 15) {
        //                 rejectUpdate = true;
        //             }
        //         }
        //         if(!rejectUpdate && estimatedPose.isPresent()) {
        //             swerveDrive.addVisionMeasurement(
        //                 estimatedPose.get().estimatedPose.toPose2d(),
        //                 result.getTimestampSeconds(),
        //                 VecBuilder.fill(
        //                     maxAmbiguity*20,
        //                     maxAmbiguity*20,
        //                     maxAmbiguity*10
        //                 )
        //             );
        //         }
        //     }

            

        //     // if (Math.abs(getRobotRelativeSpeeds().omegaRadiansPerSecond) < 4 * Math.PI || (Math.abs(getRobotRelativeSpeeds().vxMetersPerSecond) < 0.75 && Math.abs(getRobotRelativeSpeeds().vyMetersPerSecond) < 0.75)) {
        //     //     camera.photonCamera.getAllUnreadResults().forEach((result) -> {
        //     //         if(result.hasTargets()) {
        //     //             boolean rejectUpdate = false;
        //     //             double maxAmbiguity = 0;
        //     //             var targets = result.getTargets();
        //     //             for(var target : targets) {
        //     //                 double ambiguity = target.getPoseAmbiguity();
        //     //                 if(ambiguity > 0.3)
        //     //                     rejectUpdate = true;    
        //     //                 else if (ambiguity > maxAmbiguity)
        //     //                     maxAmbiguity = ambiguity;
        //     //                 // we hate the barge tags
        //     //                 if (target.getFiducialId() == 4 || target.getFiducialId() == 5 || target.getFiducialId() == 14 || target.getFiducialId() == 15) {
        //     //                     rejectUpdate = true;
        //     //                 }
        //     //             }
        //     //             Optional<EstimatedRobotPose> estimatedPose = camera.photonPoseEstimator.update(result);
        //     //             if(!rejectUpdate && estimatedPose.isPresent()) {
        //     //                 swerveDrive.addVisionMeasurement(
        //     //                     estimatedPose.get().estimatedPose.toPose2d(),
        //     //                     result.getTimestampSeconds(),
        //     //                     VecBuilder.fill(
        //     //                         maxAmbiguity*5,
        //     //                         maxAmbiguity*5,
        //     //                         maxAmbiguity*5
        //     //                     )
        //     //                 );
        //     //             }
        //     //         }
        //     //     });
        //     // }
        // }
        Logger.recordOutput(getName() + "/Robot Pose", getPose());
        if (swerveDrive.getPose() == Pose2d.kZero) {
            poseAlert.set(true);
        } else {
            poseAlert.set(false);
        }
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(), 4.0,
                swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
        zeroHeading();
        //swerveDrive.swerveDrivePoseEstimator.resetPosition(getRotation2d(), swerveDrive.getModulePositions(), pose);
    }

    /**
     * Recenter the robot's position on the field to (0,0) facing forwards.
     */
    public void recenter() {
        resetOdometry(new Pose2d(new Translation2d(), new Rotation2d()));
        // zeroHeading();
    }

    public void zeroHeading() {
        // ((com.studica.frc.AHRS) swerveDrive.getGyro().getIMU()).zeroYaw();
        //swerveDrive.zeroGyro();
        // swerveDrive.setGyroOffset(swerveDrive.getGyroRotation3d());
        headingOffset = getHeading();
    }

    /**
     * Get the unadjusted heading of the robot.
     * 
     * @return the robot's heading in degrees, from 0 to 360
     */
    public double getHeading() {
        return Math.IEEEremainder(swerveDrive.getOdometryHeading().getDegrees(), 360);
        // TODO: why are we doing this?
    }

    public double getHeadingOffset() {
        return headingOffset;
    }

    /**
     * Get the rate of turn around the Z axis(yaw).
     * 
     * @return the rate of turn in degrees per second
     */
    public double getTurnRate() {
        return swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);
    }

    public Rotation2d getRotation2d() {
        return swerveDrive.getOdometryHeading();
    }

    public boolean getFlipPath() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) { // check if alliance is red or blue
            if (ally.get() == Alliance.Red) {
                return true;
            }
            if (ally.get() == Alliance.Blue) {
                return false;
            }
        }
        return false; // if we don't know then just give up and say no
    }

    /**
     * Get the robot's current pose on the field.
     * 
     * @return the robot's current pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public SwerveModuleState[] getModuleStates() {
        return swerveDrive.getStates();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother
     *                     controls.
     * @param translationY Translation in the Y direction. Cubed for smoother
     *                     controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {
            double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control out
            double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control out
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param rotation     Rotation as a value between [-1, 1] converted to radians.
     * @return Drive command.
     */
    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                    translationY.getAsDouble(),
                    rotation.getAsDouble() * Math.PI,
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void driveRobotOriented(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }


    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the robot.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity,
                swerveDrive.getOdometryHeading().minus(Rotation2d.fromDegrees(headingOffset))),
                // Rotation2d.fromDegrees(-((com.studica.frc.AHRS) swerveDrive.getGyro().getIMU()).getAngle())),// get heading directly from the IMU and invert to make CCW+
                 true,
                new Translation2d());
    }

    public void toggleModulesLocked() {
        swerveDrive.lockPose();
    }

    public void stopModules() {
        swerveDrive.drive(new ChassisSpeeds());
    }

    public Command lockModulesCommand() {
        return new InstantCommand(() -> toggleModulesLocked())
                .andThen(() -> swerveDrive.setModuleStates(getModuleStates(), false))
                .withTimeout(1.0)
                .andThen(this::stopModules);
    }

    public void configurePathPlanner() {
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotOriented, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    translationConstants, // Translation PID constants
                    thetaConstants // Rotation PID constants
                ),
                config, // RobotConfig object
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
                this // Reference to this subsystem to set requirements
        );
        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            swerveDrive.field.getObject("path").setPoses(poses);
            Logger.recordOutput("Path", poses.toArray(new Pose2d[poses.size()]));
        });
    }

}