package lib.frc706.cyberlib.commands;

import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxDriveCommand extends Command{
	private final CommandXboxController controller;
	private final SwerveSubsystem swerveSubsystem;
	private double kMaxVelTele, kDeadband, kMaxAngularVelTele;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
	public XboxDriveCommand(CommandXboxController controller, SwerveSubsystem swerveSubsystem, double kDeadband, double kMaxVelTele, double kMaxAccelTele, double kMaxAngularVelTele, double kMaxAngularAccelTele) {
        this.controller = controller;
		this.swerveSubsystem = swerveSubsystem;
		
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
		double x = -controller.getLeftY(); //invert because up is negative for some reason
		double y = -controller.getLeftX(); //invert because FOC left is +y, controller right is +y
		double rot = -controller.getRightX(); //invert because FOC CCW is +rot, controller right is +
		double accelMultiplier = controller.getRightTriggerAxis();
		x = MathUtil.applyDeadband(x, kDeadband);
        y = MathUtil.applyDeadband(y, kDeadband);
        rot = MathUtil.applyDeadband(rot, kDeadband);
		x = Math.copySign(x * x, x);
		y = Math.copySign(y * y, y);
		rot = Math.copySign(rot * rot, rot);
		x *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		y *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		rot *= MathUtil.interpolate(0.20, 1, accelMultiplier);
        x = xLimiter.calculate(x * kMaxVelTele);
        y = yLimiter.calculate(y * kMaxVelTele);
        rot = turnLimiter.calculate(rot * kMaxAngularVelTele);
		swerveSubsystem.driveFieldOriented(swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(x, y, rot));
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}