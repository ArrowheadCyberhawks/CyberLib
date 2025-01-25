package lib.frc706.cyberlib.commands;

import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxDriveCommand extends Command{
	private final CommandXboxController controller;
	private final SwerveSubsystem swerveSubsystem;
	private double kMaxVelTele, kDeadband, kMaxAngularVelTele;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
	private final Supplier<Boolean> fieldSupplier;
	public XboxDriveCommand(CommandXboxController controller, SwerveSubsystem swerveSubsystem, Supplier<Boolean> fieldOriented, double kDeadband, double kMaxVelTele, double kMaxAccelTele, double kMaxAngularVelTele, double kMaxAngularAccelTele) {
        this.controller = controller;
		this.swerveSubsystem = swerveSubsystem;
		this.fieldSupplier = fieldOriented;
        this.xLimiter = new SlewRateLimiter(kMaxAccelTele);
        this.yLimiter = new SlewRateLimiter(kMaxAccelTele);
        this.turnLimiter = new SlewRateLimiter(kMaxAngularAccelTele);
		this.kDeadband = kDeadband;
		this.kMaxVelTele = kMaxVelTele;
		this.kMaxAngularVelTele = kMaxAngularVelTele;
		addRequirements(swerveSubsystem);
    }

	@Override
	public void execute() {
		double xInput = -controller.getLeftY(); //invert because up is negative for some reason
		double yInput = -controller.getLeftX(); //invert because FOC left is +y, controller right is +y
		double rotInput = -controller.getRightX(); //invert because FOC CCW is +rot, controller right is +
		double accelMultiplier = controller.getRightTriggerAxis();
		xInput = MathUtil.applyDeadband(xInput, kDeadband);
        yInput = MathUtil.applyDeadband(yInput, kDeadband);
        rotInput = MathUtil.applyDeadband(rotInput, kDeadband);
		xInput = Math.copySign(xInput * xInput, xInput);
		yInput = Math.copySign(yInput * yInput, yInput);
		rotInput = Math.copySign(rotInput * rotInput, rotInput);
		xInput *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		yInput *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		rotInput *= MathUtil.interpolate(0.20, 1, accelMultiplier);
        double xSpeed = xLimiter.calculate(xInput * kMaxVelTele);
        double ySpeed = yLimiter.calculate(yInput * kMaxVelTele);
        double rotSpeed = turnLimiter.calculate(rotInput * kMaxAngularVelTele);
		if(fieldSupplier.get())
			swerveSubsystem.driveFieldOriented(swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(xSpeed, ySpeed, rotSpeed));
		else
			swerveSubsystem.driveRobotOriented(swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(xSpeed, ySpeed, rotSpeed));
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}