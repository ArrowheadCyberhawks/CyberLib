package lib.frc706.cyberlib.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class ToTagCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final String name;
    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


    public ToTagCommand(SwerveSubsystem swerveSubsystem, String name) {
        this.swerveSubsystem = swerveSubsystem;
        this.name = name;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(name, 0);
    }

    @Override
    public void execute() {
        //Figure out distance and angle to apriltag
        double xSpeed;
        double ySpeed;
        double turningSpeed;
        Pose2d tagPose2d;
        int tagId;

        boolean seeingTag = LimelightHelpers.getTV(name);

        if (seeingTag) {
            tagId = (int) LimelightHelpers.getFiducialID(name);
            tagPose2d = getTagPose(tagId);
        } else {
            tagId = -999;
            tagPose2d = new Pose2d();
        }
        
        double kPturning = 0.2;
        double KpDistance = 1;
        //distance to apriltag
        double xDistance =  LimelightHelpers.getTargetPose3d_RobotSpace(name).getY();
        double yDistance = LimelightHelpers.getTargetPose3d_RobotSpace(name).getX();

        //offsets
        double xOffset = 0.8;
        double yOffset = 0.8;

        double xDistanceError = xDistance - xOffset;
        double yDistanceError = yDistance - yOffset;

        //knack code which will 100% work
        // turningSpeed = -kPturning * LimelightHelpers.getTargetPose3d_RobotSpace(name).getX(); THIS ONE POINTS AT A TAG

        //turningSpeed = seeingTag ? -kpTurning * LimelightHelpers.getTargetPose3d_RobotSpace(name). : 0;


        ySpeed = seeingTag ? MathUtil.clamp(KpDistance * yDistanceError, -3, 3) : 0;
        xSpeed = seeingTag ? MathUtil.clamp(KpDistance * xDistanceError, -3, 3) : 0;
        turningSpeed = seeingTag ? (tagPose2d.getRotation().getRadians() - swerveSubsystem.getPose().getRotation().getRadians() - Math.PI) * kPturning : 0;

        //xSpeed = 0;
        //ySpeed = 0;
        turningSpeed = 0;
        
        //Output each module states to wheels
        swerveSubsystem.driveRobotOriented(swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(xSpeed, ySpeed, turningSpeed));

        Logger.recordOutput(getName() + "/xSpeed", xSpeed);
        Logger.recordOutput(getName() + "/ySpeed", ySpeed);
        Logger.recordOutput(getName() + "/turningSpeed", turningSpeed);
        Logger.recordOutput(getName() + "/tagId", tagId);
        Logger.recordOutput(getName() + "/tagPose", tagPose2d.getRotation().getRadians());
        Logger.recordOutput(getName() + "/robot rotation", swerveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static Pose2d getTagPose(int tagId) {
        field.setOrigin(OriginPosition.kBlueAllianceWallRightSide); //copied over from robot code constants, maybe move this to constants?>
        return field.getTagPose(tagId).orElse(new Pose3d()).toPose2d();
    }

    private static Rotation2d getTrajectoryAngle(Translation2d start, Translation2d end) {
        double xdist = end.getX() - start.getX();
        double ydist = end.getY() - start.getY();
        double angle = Math.atan2(ydist, xdist);
        return new Rotation2d(angle);
    }
}