package lib.frc706.cyberlib.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class ToTagCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final String name;

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

        double kPturning = 2;
        double KpDistance = 2;
        //distance to apriltag
        double xDistance =  LimelightHelpers.getTargetPose3d_RobotSpace(name).getZ();
        double yDistance = LimelightHelpers.getTargetPose3d_RobotSpace(name).getX();
        boolean seeingTag = LimelightHelpers.getTV(name);
        //offsets
        double xOffset = 0.5;
        double yOffset = 0.1;
        double xDistanceError = xDistance - xOffset;
        double yDistanceError = yDistance - yOffset;

        //Set turning speed and y speed based off of apriltag
        xSpeed = 0;
        ySpeed = 0;
        turningSpeed = seeingTag ? -kPturning * LimelightHelpers.getTargetPose3d_RobotSpace(name).getY() : 0;

        //knack code which will 100% work
        // turningSpeed = -kPturning * LimelightHelpers.getTargetPose3d_RobotSpace(name).getX(); THIS ONE POINTS AT A TAG

        //turningSpeed = seeingTag ? -kpTurning * LimelightHelpers.getTargetPose3d_RobotSpace(name). : 0;
        ySpeed = seeingTag ? MathUtil.clamp(KpDistance * yDistanceError, -3, 3) : 0;
        xSpeed = seeingTag ? MathUtil.clamp(KpDistance * xDistanceError, -3, 3) : 0;
        
        //Output each module states to wheels
        swerveSubsystem.driveRobotOriented(swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(xSpeed, ySpeed, turningSpeed));

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}