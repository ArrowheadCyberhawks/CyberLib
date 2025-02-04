package lib.frc706.cyberlib.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class ToPointCommand extends TrackPointCommand {
    
    private PIDController xController, yController;
    private double desiredDistance;

    /**
     * Command to move the robot to a location on the field using the swerve drive. When this command is done, the front of the robot
     * will be facing directly into the face of the target pose.
     * @param swerveSubsystem the swerve subsystem
     * @param targetSupplier supplies the pose we want to move to
     * @param xController the PID controller for the x axis(revolving around the target)
     * @param yController the PID controller for the y axis(distance from the target)
     * @param desiredDistance the distance we want the center of the robot to be from the target. Usually the distance from center to the front face of the robot.
     */
    public ToPointCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetSupplier, PIDController xController, PIDController yController, PIDController turningController, double desiredDistance, double maxVel, double maxAngularVel) {
        /*
         * everything in an explicit super constructor has to be static
         * so we have to do stupid hacks like set functions to null then reset them later
         * this is a terrible way to do it but it works i guess
         * 
         * SCREW YOU JAVA
         * 
         * -grant
         * 
         * what he said was 🔥 fr - nitin
         * 
         */
        super(swerveSubsystem, targetSupplier, null, null, ()->0.0, maxVel, maxAngularVel, false, turningController);
        super.setXSupplier(() -> {return -calculateXSpeed(swerveSubsystem.getPose(), targetSupplier.get());});
        super.setYSupplier(() -> {return -calculateYSpeed(swerveSubsystem.getPose(), targetSupplier.get());});
        this.xController = xController;
        this.yController = yController;
        this.desiredDistance = desiredDistance;
        //monkey ahh code
    }

    private double calculateXSpeed(Pose2d currentPose, Pose2d targetPose) {
        return xController.calculate(calculatePolarAngleTo(currentPose, targetPose), 0)/(Math.abs(TrackPointCommand.calculateAngleTo(currentPose, targetPose))+1);
    }

    private double calculateYSpeed(Pose2d currentPose, Pose2d targetPose) {
        return yController.calculate(currentPose.getTranslation().getDistance(targetPose.getTranslation()), desiredDistance)/(Math.abs(TrackPointCommand.calculateAngleTo(currentPose, targetPose))+1);
    }
    /**
     * Calculates the angle to point the robot towards the target
     * @param currentPose current pose of the robot
     * @param targetPose pose of the apriltag (or whatever else we want to point towards)
     * @return angle between the robot and the vector facing into the front of the target
     */
    private static double calculatePolarAngleTo(Pose2d currentPose, Pose2d targetPose) {
        return currentPose.getRotation().minus(targetPose.getRotation()).minus(Rotation2d.kPi).getRadians();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        //robot needs to kill itself at some point
        return super.swerveSubsystem.getPose().getTranslation().getDistance(targetSupplier.get().getTranslation()) < desiredDistance + 0.05;
    }
}