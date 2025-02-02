package lib.frc706.cyberlib.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

/**
 * Command to point towards a location on the field using the swerve drive.
 */
public class TrackPointCommand extends Command {

    protected final SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSupplier, ySupplier, accelSupplier;
    private final PIDController m_turningController;

    private final double maxVel, maxAngularVel;
    protected Supplier<Pose2d> targetSupplier;
    private final boolean controllerCorrections;

    public TrackPointCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> target,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> accelFunction, double maxVel, double maxAngularVel, boolean controllerCorrections, PIDController turningController) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSpdFunction;
        this.ySupplier = ySpdFunction;
        this.accelSupplier = accelFunction;
        this.controllerCorrections = controllerCorrections;
        this.maxVel = maxVel;
        this.maxAngularVel = maxAngularVel;
        m_turningController = turningController;
        addRequirements(swerveSubsystem);
    }

    public void setXSupplier(Supplier<Double> newSupplier) {
        xSupplier = newSupplier;
    }

    public void setYSupplier(Supplier<Double> newSupplier) {
        ySupplier = newSupplier;
    }

    public void setAccelSupplier(Supplier<Double> newSupplier) {
        accelSupplier = newSupplier;
    }

    @Override
    public void execute() {
        //Get real-time joystick inputs
        double xInput = ySupplier.get();
        double yInput = xSupplier.get();
        double accelMultiplier = accelSupplier.get();
        double xSpeed = 0, ySpeed = 0;
        if(controllerCorrections) {
            xSpeed = xInput * maxVel;
            ySpeed = yInput * maxVel;
            
        } else {
            xSpeed = MathUtil.clamp(xInput, -maxVel, maxVel);
            ySpeed = MathUtil.clamp(yInput, -maxVel, maxVel);
        }

        // MONKEY CODE (made by our fellow monkey)
        //targetAngle = Math.atan2(targetPose.getTranslation().getY()-robotPose.getTranslation().getY(), targetPose.getTranslation().getX()-robotPose.getTranslation().getX());
        
        double turningSpeed = m_turningController.calculate(calculateAngleTo(swerveSubsystem.getPose(), targetSupplier.get()));
        turningSpeed = MathUtil.clamp(turningSpeed, -maxAngularVel, maxAngularVel);

        //Output each module states to wheels
        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(xSpeed, ySpeed, turningSpeed));
    }

    /**
     * Calculate the difference between the robot's current angle and the angle required to point at a specified location.
     * 
     * @param robotPose The current pose of the robot.
     * @param targetPose The pose of the target location.
     * @return The angle difference in radians.
     */
    public static double calculateAngleTo(Pose2d robotPose, Pose2d targetPose) {
        double targetAngle = targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();
        return MathUtil.angleModulus(robotPose.getRotation().getRadians()-targetAngle);
    }

    @Override
    public void initialize() {
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