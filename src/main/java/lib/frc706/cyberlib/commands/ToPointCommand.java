package lib.frc706.cyberlib.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class ToPointCommand extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private ProfiledPIDController xController, yController, thetaController;
    private Supplier<Pose2d> targetSupplier;

    /**
     * Command to move the robot to a location on the field using the swerve drive. When this command is done, the front of the robot
     * will be facing directly into the face of the target pose.
     * @param swerveSubsystem the swerve subsystem
     * @param xController the PID controller for the x axis
     * @param yController the PID controller for the y axis
     * @param turningController the PID controller for the rotation of the robot (used for angling the robot towards the target)
     * @param targetSupplier supplies the pose we want to move to
     */
    public ToPointCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetSupplier, ProfiledPIDController xController, ProfiledPIDController yController, ProfiledPIDController turningController, double translationTolerance, double rotationTolerance) {
        this.swerveSubsystem = swerveSubsystem;
        this.xController = xController;
        this.yController = yController;
        thetaController = turningController;
        this.xController.setTolerance(translationTolerance);
        this.yController.setTolerance(translationTolerance);
        thetaController.setTolerance(rotationTolerance);
        this.targetSupplier = targetSupplier;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveSubsystem.getPose();
        Pose2d targetPose = targetSupplier.get();
        if (targetPose.getX() != xController.getGoal().position) {
            xController.setGoal(targetPose.getX());
        }
        if (targetPose.getY() != yController.getGoal().position) {
            yController.setGoal(targetPose.getY());
        }
        if (targetPose.getRotation().getRadians() != thetaController.getGoal().position) {
            thetaController.setGoal(targetPose.getRotation().getRadians());
        }
        double xSpeed = xController.calculate(targetPose.getX() - currentPose.getX());
        double ySpeed = yController.calculate(targetPose.getY() - currentPose.getY());
        double thetaSpeed = thetaController.calculate(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        swerveSubsystem.driveFieldOriented(speeds);
        System.out.println("X: " + xSpeed + " Y: " + ySpeed + " Theta: " + thetaSpeed);
    }

    @Override
    public void initialize() {
        if (targetSupplier == null) {
           return;
        }
        xController.setGoal(targetSupplier.get().getX());
        yController.setGoal(targetSupplier.get().getY());
        thetaController.setGoal(targetSupplier.get().getRotation().getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        //robot needs to kill itself at some point
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
}