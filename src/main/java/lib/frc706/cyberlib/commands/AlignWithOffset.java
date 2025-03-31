package lib.frc706.cyberlib.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class AlignWithOffset extends Command {
    private PIDController xController, yController, thetaController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private double tagID = -1;
    private SwerveSubsystem swerveSubsystem;

    private double xOffset;
    private double yOffset;

    private double xTolerance;
    private double yTolerance;

    public AlignWithOffset(boolean isRightScore, SwerveSubsystem swerveSubsystem, Translation2d offset, String name) {
        xController = new PIDController(1, 0, 0);
        yController = new PIDController(1, 0, 0);
        thetaController = new PIDController(1, 0, 0);
        this.isRightScore = isRightScore;
        this.swerveSubsystem = swerveSubsystem;
        xOffset = offset.getX();
        yOffset = offset.getY();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        thetaController.setSetpoint(30);
        thetaController.setTolerance(0.01);

        xController.setSetpoint(xOffset);
        xController.setTolerance(xTolerance);

        yController.setSetpoint(isRightScore ? yOffset : -yOffset);
        yController.setTolerance(yTolerance);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
            this.dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");

            double xSpeed = xController.calculate(positions[2]);
            double ySpeed = -yController.calculate(positions[0]);
            double turningSpeed = -thetaController.calculate(positions[4]);

            swerveSubsystem.driveRobotOriented(swerveSubsystem.swerveDrive.swerveController
                    .getRawTargetSpeeds(yController.getError() < 0.1 ? xSpeed : 0, ySpeed, turningSpeed));

            if (!thetaController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            swerveSubsystem.stopModules();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long
        // as it gets a tag in the camera
        return thetaController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint();
    }
}