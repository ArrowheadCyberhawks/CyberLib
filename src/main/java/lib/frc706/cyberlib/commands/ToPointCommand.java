package lib.frc706.cyberlib.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class ToPointCommand extends TrackPointCommand {
    
    private static PIDController xController, yController;
    private double desiredDistance;

    /**
     * Command to move the robot to a location on the field using the swerve drive. When this command is done, the front of the robot
     * will be facing directly into the face of the target pose.
     * @param swerveSubsystem the swerve subsystem
     * @param xController the PID controller for the x axis(revolving around the target)
     * @param yController the PID controller for the y axis(distance from the target)
     * @param turningController the PID controller for the rotation of the robot (used for angling the robot towards the target)
     * @param targetSupplier supplies the pose we want to move to
     * @param desiredDistance the distance we want the center of the robot to be from the target. Usually the distance from center to the front face of the robot.
     * @param maxVel the maximum velocity of the robot in m/s
     * @param maxAngularVel the maximum angular velocity of the robot in rad/s
     */
    public ToPointCommand(SwerveSubsystem swerveSubsystem, PIDController xController, PIDController yController, PIDController turningController, Supplier<Pose2d> targetSupplier, double desiredDistance, double maxVel, double maxAngularVel) {
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
        super(swerveSubsystem, turningController, targetSupplier, null, null, ()-> 0.0, maxVel, maxAngularVel);
        super.setXSupplier(() -> {return -calculateXSpeed(swerveSubsystem.getPose(), targetSupplier.get());});
        super.setYSupplier(() -> {return -calculateYSpeed(swerveSubsystem.getPose(), targetSupplier.get());});
        ToPointCommand.xController = xController;
        ToPointCommand.yController = yController;
        this.desiredDistance = desiredDistance;
        //monkey ahh code
    }

    /**
     * WARNING: ONLY USE THIS CONSTRUCTOR IF THE OTHER ONE HAS ALREADY BEEN USED AT LEAST ONCE!!!
     * THE OTHER CONSTRUCTOR SETS STATIC VARIABLES WHICH ARE REQUIRED FOR THIS ONE TO WORK!
     * 
     * @param targetSupplier Supplies the pose we want to move to
     * @param desiredDistance The distance we want the center of the robot to be from the target. Usually the distance from center to the front face of the robot.
     */
    public ToPointCommand(Supplier<Pose2d> targetSupplier, double desiredDistance) {
        super(targetSupplier, null, null); // i hate this so much
        super.setXSupplier(() -> {return -calculateXSpeed(swerveSubsystem.getPose(), targetSupplier.get());});
        super.setYSupplier(() -> {return -calculateYSpeed(swerveSubsystem.getPose(), targetSupplier.get());});
        this.desiredDistance = desiredDistance;
        if(xController == null) {
            throw new NullPointerException("xController is null!");
        } else if(yController == null) {
            throw new NullPointerException("yController is null!");
        }
    }

    /**
     * Calculates the horizontal speed of the robot around a circle centered at the target pose 
     * such that the robot will lie on the line normal to the face of the target
     * 
     * @param currentPose current pose of the robot
     * @param targetPose pose of the apriltag (or whatever else we want to point towards)
     */
    private double calculateXSpeed(Pose2d currentPose, Pose2d targetPose) {
        double thetaError = calculatePolarAngleTo(currentPose, targetPose); // get the distance we need to travel around the circle
        double xSpeed = xController.calculate(thetaError, 0); // calculate the speed we need to travel with PID
        xSpeed /= (Math.abs(TrackPointCommand.calculateAngleTo(currentPose, targetPose))+1); // slow down if we're not facing the target
        return xSpeed;
    }

    private double calculateYSpeed(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation()); // get the distance from the target
        double ySpeed = yController.calculate(distance, desiredDistance); // calculate the speed we need to travel with PID
        ySpeed /= (Math.abs(TrackPointCommand.calculateAngleTo(currentPose, targetPose))+1); // slow down if we're not facing the target
        return ySpeed;
    }
    /**
     * Calculates the angle to point the robot towards the target
     * @param currentPose current pose of the robot
     * @param targetPose pose of the apriltag (or whatever else we want to point towards)
     * @return angle between the robot and the vector facing into the front of the target
     */
    private static double calculatePolarAngleTo(Pose2d currentPose, Pose2d targetPose) {
        Rotation2d thetaError = currentPose.getRotation().minus(targetPose.getRotation());
                        // TODO: ?!?!?!?!?!?!?!?!?!?!??!?!
        return thetaError.minus(Rotation2d.fromDegrees(69)).getRadians();
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
        return swerveSubsystem.getPose().getTranslation().getDistance(targetSupplier.get().getTranslation()) < desiredDistance + 0.05;
    }
}