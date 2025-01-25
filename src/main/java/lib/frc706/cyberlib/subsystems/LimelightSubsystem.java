package lib.frc706.cyberlib.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private String[] names;
    private SwerveSubsystem swerveSubsystem;
    //private NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    //private NetworkTableEntry[] botposes;

    public LimelightSubsystem(SwerveSubsystem swerveSubsystem, String... names) {
        this.names = names;
        this.swerveSubsystem = swerveSubsystem;

        /*
         * botposes = new NetworkTableEntry[names.length];
         * for (int i = 0; i < names.length; i++) {
         * botposes[i] = ntinst.getTable(names[i]).getEntry("botpose");
         * }
         */
    }

    @Override
    public void periodic() {
        swerveSubsystem.swerveDrive.updateOdometry();
        for (String name : names) {
            boolean useMegaTag2 = false; // set to false to use MegaTag1
            boolean doRejectUpdate = false;
            
            if (!useMegaTag2) {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
                
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
                            .setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 0.5));
                    swerveSubsystem.swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
                            mt1.pose,
                            mt1.timestampSeconds);
                    // swerveSubsystem.swerveDrive.updateOdometry();
                }
            } else if (useMegaTag2) {
                LimelightHelpers.SetRobotOrientation(name, swerveSubsystem.swerveDrive.swerveDrivePoseEstimator
                        .getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

                if (Math.abs(swerveSubsystem.swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 720) { //if the angular velocity is too high it disregards all megatag localizatoin code
                    doRejectUpdate = true;
                }
                if (mt2.tagCount == 0) {
                    doRejectUpdate = true;
                }
                if (!doRejectUpdate) {
                    swerveSubsystem.swerveDrive.swerveDrivePoseEstimator
                            .setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
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
