package lib.frc706.cyberlib.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ToPointCommand extends Command {
    
    private final SwerveSubsystem swerveSubsystem;

    private PIDController xController, yController, thetaController;
    private final LoggedNetworkNumber kPDrive = new LoggedNetworkNumber("ToPoint/kPDrive", 7);
    private final LoggedNetworkNumber kPTheta = new LoggedNetworkNumber("ToPoint/kPTheta", 6);

    private final LoggedNetworkNumber kIDrive = new LoggedNetworkNumber("ToPoint/kIDrive", 0);

    private final LoggedNetworkNumber kDDrive = new LoggedNetworkNumber("ToPoint/kDDrive", 0.05);//0.01

    private final LoggedNetworkNumber kDriveMaxVel = new LoggedNetworkNumber("ToPoint/kDriveMaxVel", 0.5);
    private final LoggedNetworkNumber kDriveMaxAccel = new LoggedNetworkNumber("ToPoint/kDriveMaxAccel", 1);

    private final LoggedNetworkNumber kThetaMaxVel = new LoggedNetworkNumber("ToPoint/kThetaMaxVel", Math.PI);
    private final LoggedNetworkNumber kThetaMaxAccel = new LoggedNetworkNumber("ToPoint/kThetaMaxAccel", 2 * Math.PI);

    private final LoggedNetworkNumber kDriveTolerance = new LoggedNetworkNumber("ToPoint/kDriveTolerance", 0.000001);
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
        xController = new PIDController(kPDrive.get(), kIDrive.get(), kDDrive.get());
        yController = new PIDController(kPDrive.get(), kIDrive.get(), kDDrive.get());
        thetaController = new PIDController(kPTheta.get(), 0, 0);
        xController.setTolerance(kDriveTolerance.get());
        yController.setTolerance(kDriveTolerance.get());
        thetaController.setTolerance(kThetaTolerance.get());

        // advantagekit stuff

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // updateConstants();
        // Pose2d currentPose = swerveSubsystem.getPose();
        Double[] poseArray = SmartDashboard.getNumberArray("Field/Robot", new Double[] {0.0, 0.0, 0.0});
        Pose2d currentPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[2]));
        // Logger.recordOutput(getName() + "/xPosition", currentPose.getX());
        // Logger.recordOutput(getName() + "/yPosition", currentPose.getY());
        // Logger.recordOutput(getName() + "/thetaPosition", currentPose.getRotation().getRadians());
        Pose2d targetPose = targetSupplier.get();
        if (targetPose.getX() != xController.getSetpoint()) {
            xController.setSetpoint(targetPose.getX());
        }
        if (targetPose.getY() != yController.getSetpoint()) {
            yController.setSetpoint(targetPose.getY());
        }
        if (targetPose.getRotation().getRadians() != thetaController.getSetpoint()) {
            thetaController.setSetpoint(targetPose.getRotation().getRadians());
        }
        double xSpeed = MathUtil.clamp(xController.calculate(currentPose.getX()), -kDriveMaxVel.get(), kDriveMaxVel.get());
        double ySpeed = MathUtil.clamp(yController.calculate(currentPose.getY()), -kDriveMaxVel.get(), kDriveMaxVel.get());
        double thetaSpeed = MathUtil.clamp(thetaController.calculate(currentPose.getRotation().getRadians()), -kThetaMaxVel.get(), kThetaMaxVel.get());
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        swerveSubsystem.swerveDrive.driveFieldOriented(speeds);

        // more advantagekit stuff
        // Logger.recordOutput(getName() + "/xSetpoint", xController.getSetpoint());
        // Logger.recordOutput(getName() + "/ySetpoint", yController.getSetpoint());
        // Logger.recordOutput(getName() + "/thetaSetpoint", thetaController.getSetpoint());
        // Logger.recordordOutput(getName() + "/xSpeed", xSpeed);
        // Logger.recordOutput(getName() + "/ySpeed", ySpeed);
        // Logger.recordOutput(getName() + "/thetaSpeed", thetaSpeed);
        // Logger.recordOutput(getName() + "/xError", xController.getPositionError());
        // Logger.recordOutput(getName() + "/yError", yController.getPositionError());
        // Logger.recordOutput(getName() + "/thetaError", thetaController.getPositionError());
        // Logger.recordOutput(getName() + "/targetPose", targetPose);
        // Logger.recordOutput(getName() + "/currentPose", currentPose);
        // Logger.recordOutput(getName() + "/realXError", targetPose.getX() - currentPose.getX());
        // Logger.recordOutput(getName() + "/realYError", targetPose.getY() - currentPose.getY());
        // // Logger.recordOutput(getName() + "/xGoal", xController.getGoal().position);
        // // Logger.recordOutput(getName() + "/yGoal", yController.getGoal().position);
        // // Logger.recordOutput(getName() + "/thetaGoal", thetaController.getGoal().position);
    }

    @Override
    public void initialize() {
        // xController = new PIDController(kPDrive.get(), kIDrive.get(), kDDrive.get(), new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get()));
        // yController = new PIDController(kPDrive.get(), kIDrive.get(), kDDrive.get(), new Constraints(kDriveMaxVel.get(), kDriveMaxAccel.get()));
        // thetaController = new PIDController(kPTheta.get(), 0, 0, new Constraints(kThetaMaxVel.get(), kThetaMaxAccel.get()));
        xController.setTolerance(kDriveTolerance.get());
        yController.setTolerance(kDriveTolerance.get());
        thetaController.setTolerance(kThetaTolerance.get());
        xController.setSetpoint(targetSupplier.get().getX());
        yController.setSetpoint(targetSupplier.get().getY());
        thetaController.setSetpoint(targetSupplier.get().getRotation().getRadians());
        xController.reset();
        yController.reset();
        thetaController.reset();
        if (targetSupplier == null) {
           return;
        }
    }

    /**
     * Checks if any networktables inputs have changed and updates the PID controllers accordingly.
     */
    private void updateConstants() {
            // absolute unit of an if statement
        if (xController.getP() != kPDrive.get() ||
            yController.getP() != kPDrive.get() ||
            xController.getI() != kIDrive.get() ||
            yController.getI() != kIDrive.get() ||
            xController.getD() != kDDrive.get() ||
            yController.getD() != kDDrive.get() ||
            thetaController.getP() != kPTheta.get() || 
            xController.getPositionTolerance() != kDriveTolerance.get() || 
            yController.getPositionTolerance() != kDriveTolerance.get() || 
            thetaController.getPositionTolerance() != kThetaTolerance.get()) {

            xController.setP(kPDrive.get());
            yController.setP(kPDrive.get());
            thetaController.setP(kPTheta.get());

            xController.setI(kIDrive.get());
            yController.setI(kIDrive.get());

            xController.setD(kDDrive.get());
            yController.setD(kDDrive.get());

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
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}