package lib.frc706.cyberlib.commands;

import edu.wpi.first.math.MathUtil;
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
        double xSpeed = 0;
        double ySpeed;
        double turningSpeed;
        double kPturning = 1;
        double KpDistance = 2;
        double distance =  LimelightHelpers.getTargetPose3d_RobotSpace(name).getZ();
        double desiredDistance = 0.5;
        double distance_error = distance-desiredDistance;

        //Set turning speed and y speed based off of apriltag
        ySpeed = 0;
        turningSpeed = kPturning * LimelightHelpers.getTargetPose3d_RobotSpace(name).getX();
        xSpeed = LimelightHelpers.getTV(name) ? MathUtil.clamp(KpDistance*distance_error, -3, 3) : 0;
        
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