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

    protected static SwerveSubsystem swerveSubsystem;
    private static PIDController m_turningController;
    private static double maxVel = Double.MIN_VALUE;
    private static double maxAngularVel = Double.MIN_VALUE;

    private Supplier<Double> xSupplier, ySupplier, accelSupplier;
    protected Supplier<Pose2d> targetSupplier;
    private final boolean controllerCorrections;

    public TrackPointCommand(SwerveSubsystem swerveSubsystem, PIDController turningController, Supplier<Pose2d> targetSupplier,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> accelFunction, double maxVel, double maxAngularVel) {
        TrackPointCommand.swerveSubsystem = swerveSubsystem;
        this.targetSupplier = targetSupplier;
        this.xSupplier = xSpdFunction;
        this.ySupplier = ySpdFunction;
        this.accelSupplier = accelFunction;
        this.controllerCorrections = true;
        TrackPointCommand.maxVel = maxVel;
        TrackPointCommand.maxAngularVel = maxAngularVel;
        m_turningController = turningController;
        addRequirements(swerveSubsystem);
    }

    public TrackPointCommand(SwerveSubsystem swerveSubsystem, PIDController turningController, Supplier<Pose2d> targetSupplier,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, double maxVel, double maxAngularVel) {
        TrackPointCommand.swerveSubsystem = swerveSubsystem;
        this.targetSupplier = targetSupplier;
        this.xSupplier = xSpdFunction;
        this.ySupplier = ySpdFunction;
        this.accelSupplier = ()->1.0;
        this.controllerCorrections = false;
        TrackPointCommand.maxVel = maxVel;
        TrackPointCommand.maxAngularVel = maxAngularVel;
        m_turningController = turningController;
        addRequirements(swerveSubsystem);
    }

    /**
     * WARNING: DO NOT USE THIS CONSTRUCTOR UNLESS AT LEAST ONE OF THE FULL CONSTRUCTORS HAS BEEN USED FIRST!!!
     * THE OTHER CONSTRUCTORS INITIALIZE STATIC VARIABLES WHICH WILL CAUSE THIS CONSTRUCTOR TO THROW A NULL POINTER EXCEPTION IF NOT USED FIRST!
     * 
     * Partial constructor for TrackPointCommand. Supplying an acceleration function is optional, and will enable controller corrections if provided.
     * 
     * @param targetSupplier Supplies the pose we want to point towards.
     * @param xSpdFunction Supplies the x speed of the robot in m/s.
     * @param ySpdFunction Supplies the y speed of the robot in m/s.
     * @param accelFunction Supplies the interpolation factor for the speed of the robot.
     */
    public TrackPointCommand(Supplier<Pose2d> targetSupplier, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> accelFunction) {
        this(swerveSubsystem, m_turningController, targetSupplier, xSpdFunction, ySpdFunction, accelFunction, maxVel, maxAngularVel);
        if(swerveSubsystem == null) {
            throw new NullPointerException("SwerveSubsystem is null!");
        } else if (maxVel == Double.MIN_VALUE) {
            throw new NullPointerException("maxVel is not set!");
        } else if (maxAngularVel == Double.MIN_VALUE) {
            throw new NullPointerException("maxAngularVel is not set!");
        } else if (m_turningController == null) {
            throw new NullPointerException("m_turningController is null!");
        }
    }

    /**
     * WARNING: DO NOT USE THIS CONSTRUCTOR UNLESS AT LEAST ONE OF THE FULL CONSTRUCTORS HAS BEEN USED FIRST!!!
     * THE OTHER CONSTRUCTORS INITIALIZE STATIC VARIABLES WHICH WILL CAUSE THIS CONSTRUCTOR TO THROW A NULL POINTER EXCEPTION IF NOT USED FIRST!
     * 
     * Partial constructor for TrackPointCommand with controller corrections disabled.
     * 
     * @param targetSupplier Supplies the pose we want to point towards.
     * @param xSpdFunction Supplies the x speed of the robot in m/s.
     * @param ySpdFunction Supplies the y speed of the robot in m/s.
     */
    public TrackPointCommand(Supplier<Pose2d> targetSupplier, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction) {
        this(swerveSubsystem, m_turningController, targetSupplier, xSpdFunction, ySpdFunction, maxVel, maxAngularVel);
        if(swerveSubsystem == null) {
            throw new NullPointerException("SwerveSubsystem is null!");
        } else if (maxVel == Double.MIN_VALUE) {
            throw new NullPointerException("maxVel is not set!");
        } else if (maxAngularVel == Double.MIN_VALUE) {
            throw new NullPointerException("maxAngularVel is not set!");
        } else if (m_turningController == null) {
            throw new NullPointerException("m_turningController is null!");
        }
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