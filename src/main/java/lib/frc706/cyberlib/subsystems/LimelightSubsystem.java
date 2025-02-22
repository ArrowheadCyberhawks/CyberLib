package lib.frc706.cyberlib.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private String[] names;
    private SwerveSubsystem swerveSubsystem;
    
    private boolean usePose;
    private boolean useMegaTag2;
    //private NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    //private NetworkTableEntry[] botposes;

    public LimelightSubsystem(SwerveSubsystem swerveSubsystem, boolean usePose, boolean useMegaTag2, String... names) {
        this.names = names;
        this.swerveSubsystem = swerveSubsystem;
        this.useMegaTag2 = useMegaTag2;
        this.usePose = usePose;
        /*
         * botposes = new NetworkTableEntry[names.length];
         * for (int i = 0; i < names.length; i++) {
         * botposes[i] = ntinst.getTable(names[i]).getEntry("botpose");
         * }
         */
    }

    @Override
    public void periodic() {
        if (usePose) {
            poseUpdate();
        } else {
            tagOffset();
        }
        
    }

    public void tagOffset() {
        for (String name : names) {
            //not really doing anything here uhhhhhhh
            //sounds like a later problem
        }
    }

    public void poseUpdate() {
        swerveSubsystem.swerveDrive.updateOdometry();
        for (String name : names) {
            boolean doRejectUpdate = false;
            LimelightHelpers.SetRobotOrientation(name,swerveSubsystem.swerveDrive.getOdometryHeading().getDegrees(), 0, 0, 0, 0, 0);
            if (!useMegaTag2) {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
                if (mt1 == null) {
                    continue;
                }
                
                if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                    if (mt1.rawFiducials[0].ambiguity > .7) {
                        doRejectUpdate = true;
                    }
                    if (mt1.rawFiducials[0].distToCamera > 3) {
                        doRejectUpdate = true;
                    }
                }
                if (mt1.tagCount == 0) {
                    doRejectUpdate = true;
                }

                if (!doRejectUpdate) {
                    swerveSubsystem.swerveDrive.swerveDrivePoseEstimator
                            .setVisionMeasurementStdDevs(VecBuilder.fill(5, 5, 1.5));// nitin do not touch this number again
                    swerveSubsystem.swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
                            mt1.pose,
                            mt1.timestampSeconds);
                }
            } else if (useMegaTag2) {
                LimelightHelpers.SetRobotOrientation(name,swerveSubsystem.swerveDrive.getOdometryHeading().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
                if (mt2 == null) {
                    continue;
                }
                // System.out.println(swerveSubsystem.getRotation2d().getDegrees() + " " + swerveSubsystem.getHeading());

                if (Math.abs(swerveSubsystem.swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 720) { //if the angular velocity is too high it disregards all megatag localizatoin code
                    doRejectUpdate = true;
                }
                if (mt2.tagCount == 0) {
                    doRejectUpdate = true;
                }
                if (!doRejectUpdate) {
                    swerveSubsystem.swerveDrive.swerveDrivePoseEstimator
                            .setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 1));
                    swerveSubsystem.swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
                            mt2.pose,
                            mt2.timestampSeconds);
                    // swerveSubsystem.swerveDrive.updateOdometry();
                }
            }
        }
    }

    /*
     * LimelightResults result;
     * for(int i = 0; i < names.length; i++) {
     * result = LimelightHelpers.getLatestResults("limelight");
     * double[] poseArray = botposes[i].getDoubleArray(new double[10]);
     * System.out.println(poseArray[0]);
     * // System.out.println(result.getBotPose2d_wpiBlue().getX());
     * //
     * swerveSubsystem.swerveDrive.addVisionMeasurement(result.getBotPose2d_wpiBlue(
     * ), result.timestamp_LIMELIGHT_publish);
     * swerveSubsystem.swerveDrive.addVisionMeasurement(new Pose2d(poseArray[0],
     * poseArray[1],
     * Rotation2d.fromDegrees(poseArray[5])), result.timestamp_LIMELIGHT_publish);
     * swerveSubsystem.swerveDrive.updateOdometry();
     * }
     */
}
