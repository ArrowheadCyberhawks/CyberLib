package lib.frc706.cyberlib.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ToPointCommand extends Command {
    
    private final SwerveSubsystem swerveSubsystem;

    private ProfiledPIDController xController, yController, thetaController;
    private final LoggedNetworkNumber kPDrive = new LoggedNetworkNumber("ToPoint/kPDrive", 5);
    private final LoggedNetworkNumber kPTheta = new LoggedNetworkNumber("ToPoint/kPTheta", 5);

    private final LoggedNetworkNumber kDriveMaxVel = new LoggedNetworkNumber("ToPoint/kDriveMaxVel", 1);
    private final LoggedNetworkNumber kDriveMaxAccel = new LoggedNetworkNumber("ToPoint/kDriveMaxAccel", 1);

    private final LoggedNetworkNumber kThetaMaxVel = new LoggedNetworkNumber("ToPoint/kThetaMaxVel", Math.PI);
    private final LoggedNetworkNumber kThetaMaxAccel = new LoggedNetworkNumber("ToPoint/kThetaMaxAccel", 2 * Math.PI);

    private final LoggedNetworkNumber kDriveTolerance = new LoggedNetworkNumber("ToPoint/kDriveTolerance", 0.01);
    private final LoggedNetworkNumber kThetaTolerance = new LoggedNetworkNumber("ToPoint/kThetaTolerance", 0.01);

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
    public ToPointCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetSupplier = targetSupplier;

        //set up PID controllers
        xController = new ProfiledPIDController(kPDrive.get(), 0, 0, new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get()));
        yController = new ProfiledPIDController(kPDrive.get(), 0, 0, new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get()));
        thetaController = new ProfiledPIDController(kPTheta.get(), 0, 0, new Constraints(kThetaMaxVel.get(), kThetaMaxAccel.get()));
        xController.setTolerance(kDriveTolerance.get());
        yController.setTolerance(kDriveTolerance.get());
        thetaController.setTolerance(kThetaTolerance.get());

        // advantagekit stuff

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        updateConstants();
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
        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        swerveSubsystem.driveFieldOriented(speeds);

        // more advantagekit stuff
        Logger.recordOutput(getName() + "/xSetpoint", xController.getSetpoint().position);
        Logger.recordOutput(getName() + "/ySetpoint", yController.getSetpoint().position);
        Logger.recordOutput(getName() + "/thetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput(getName() + "/xSpeed", xSpeed);
        Logger.recordOutput(getName() + "/ySpeed", ySpeed);
        Logger.recordOutput(getName() + "/thetaSpeed", thetaSpeed);
        Logger.recordOutput(getName() + "/xError", xController.getPositionError());
        Logger.recordOutput(getName() + "/yError", yController.getPositionError());
        Logger.recordOutput(getName() + "/thetaError", thetaController.getPositionError());
        Logger.recordOutput(getName() + "/targetPose", targetPose);
        Logger.recordOutput(getName() + "/currentPose", currentPose);
        Logger.recordOutput(getName() + "/realXError", targetPose.getX() - currentPose.getX());
        Logger.recordOutput(getName() + "/realYError", targetPose.getY() - currentPose.getY());
        Logger.recordOutput(getName() + "/xGoal", xController.getGoal().position);
        Logger.recordOutput(getName() + "/yGoal", yController.getGoal().position);
        Logger.recordOutput(getName() + "/thetaGoal", thetaController.getGoal().position);
        Logger.recordOutput(getName() + "/xPosition", currentPose.getX());
        Logger.recordOutput(getName() + "/yPosition", currentPose.getY());
        Logger.recordOutput(getName() + "/thetaPosition", currentPose.getRotation().getRadians());
    }

    @Override
    public void initialize() {
        if (targetSupplier == null) {
           return;
        }
        xController = new ProfiledPIDController(kPDrive.get(), 0, 0, new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get()));
        yController = new ProfiledPIDController(kPDrive.get(), 0, 0, new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get()));
        thetaController = new ProfiledPIDController(kPTheta.get(), 0, 0, new Constraints(kThetaMaxVel.get(), kThetaMaxAccel.get()));
        xController.setTolerance(kDriveTolerance.get());
        yController.setTolerance(kDriveTolerance.get());
        thetaController.setTolerance(kThetaTolerance.get());
        xController.setGoal(targetSupplier.get().getX());
        yController.setGoal(targetSupplier.get().getY());
        thetaController.setGoal(targetSupplier.get().getRotation().getRadians());
        xController.reset(swerveSubsystem.getPose().getX());
        yController.reset(swerveSubsystem.getPose().getY());
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians());
    }

    /**
     * Checks if any networktables inputs have changed and updates the PID controllers accordingly.
     */
    private void updateConstants() {
            // absolute unit of an if statement
        if (xController.getP() != kPDrive.get() ||
            yController.getP() != kPDrive.get() ||
            thetaController.getP() != kPTheta.get() || 
            !xController.getConstraints().equals(new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get())) || 
            !yController.getConstraints().equals(new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get())) || 
            !thetaController.getConstraints().equals(new Constraints(kThetaMaxVel.get(), kThetaMaxAccel.get())) || 
            xController.getPositionTolerance() != kDriveTolerance.get() || 
            yController.getPositionTolerance() != kDriveTolerance.get() || 
            thetaController.getPositionTolerance() != kThetaTolerance.get()) {

            xController.setP(kPDrive.get());
            yController.setP(kPDrive.get());
            thetaController.setP(kPTheta.get());

            Constraints driveConstraints = new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get());
            xController.setConstraints(driveConstraints);
            yController.setConstraints(driveConstraints);

            Constraints thetaConstraints = new Constraints(kThetaMaxVel.get(), kThetaMaxAccel.get());
            thetaController.setConstraints(thetaConstraints);

            xController.setTolerance(kDriveTolerance.get());
            yController.setTolerance(kDriveTolerance.get());
            thetaController.setTolerance(kThetaTolerance.get());
        }
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