package lib.frc706.cyberlib.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static lib.frc706.cyberlib.Constants.SwerveModule.*;


public class SwerveModule {
    public final double MAX_VELOCITY_METERS_PER_SECOND;
    private final double DRIVE_ROT2METER;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final PIDController turningPidController;
    private final AnalogInput absoluteEncoder;

    private final ModulePosition modulePosition;

    private boolean isLocked = false;
    private final boolean absoluteEncoderInverted;

    private double absoluteEncoderOffset;
    private double kPTurning, kITurning, kDTurning;

    private final ShuffleboardLayout moduleLayout;
    private final GenericEntry absoluteEncoderEntry;
    private final GenericEntry driveEncoderEntry;

    public enum ModulePosition {
        FL (-45.0),
        FR (45.0),
        BL (45.0),
        BR (-45.0);

        private final double lockAngle;

        ModulePosition(double lockAngle) {
            this.lockAngle = lockAngle;
        }

        Rotation2d getLockRotation() {
            return Rotation2d.fromDegrees(lockAngle);
        }
    }

    public SwerveModule(ModuleType moduleType, ModulePosition modulePosition, int driveMotorID, int turningMotorID, int absoluteEncoderPort,
                        double absoluteEncoderOffset, boolean driveMotorInverted, boolean turningMotorInverted, boolean absoluteEncoderInverted) {

        DRIVE_ROT2METER = moduleType.getDriveRot2Meter();
        MAX_VELOCITY_METERS_PER_SECOND = moduleType.getMaxVelocity();

        this.modulePosition = modulePosition;
        this.absoluteEncoderInverted = absoluteEncoderInverted;
        Preferences.setDouble(modulePosition.name() + absEncoderOffsetKey, absoluteEncoderOffset);
        initPreferences();
        loadPreferences();
        

        absoluteEncoder = new AnalogInput(absoluteEncoderPort);

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.setInverted(driveMotorInverted);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        turningMotor.setInverted(turningMotorInverted);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(DRIVE_ROT2METER);
        driveEncoder.setVelocityConversionFactor(DRIVE_ROT2METER / 60);
        turningEncoder.setPositionConversionFactor(moduleType.getSteerReduction() * 2 * Math.PI);
        turningEncoder.setVelocityConversionFactor(moduleType.getSteerReduction() * 2 * Math.PI / 60);
        turningPidController = new PIDController(kPTurning, kITurning, kDTurning); 
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

        moduleLayout = Shuffleboard.getTab("SwerveDrive").getLayout("SwerveDrive", BuiltInLayouts.kGrid).getLayout("SwerveModule" + modulePosition.name(), BuiltInLayouts.kList);
        moduleLayout.add("CAUTION: Reset Absolute Encoder Offset", rezeroCommand()).withWidget(BuiltInWidgets.kCommand);
        absoluteEncoderEntry = moduleLayout.add("Absolute Encoder Degrees", getAbsoluteTurningAngle().getDegrees()).withWidget(BuiltInWidgets.kGyro).getEntry();
        driveEncoderEntry = moduleLayout.add("Drive Encoder Meters", driveEncoder).withWidget(BuiltInWidgets.kEncoder).getEntry();
    }

    public SwerveModule(ModuleType moduleType, ModulePosition modulePosition, int driveMotorID, int turningMotorID, int absoluteEncoderPort, int absoluteEncoderOffset) {
        this(moduleType, modulePosition, driveMotorID, turningMotorID, absoluteEncoderPort, absoluteEncoderOffset, false, false, false);
    }

    /**
     * Initializes preferences for this module in NetworkTables if they don't exist.
     */
    private void initPreferences() {
        Preferences.initDouble(modulePosition.name() + absEncoderOffsetKey, defaultAbsoluteEncoderOffset);
        Preferences.initDouble(modulePosition.name() + kPTurningKey, defaultkPTurning);
        Preferences.initDouble(modulePosition.name() + kITurningKey, defaultkITurning);
        Preferences.initDouble(modulePosition.name() + kDTurningKey, defaultkDTurning);
    }

    /**
     * Loads preferences for this module from NetworkTables.
     */
    private void loadPreferences() {
        absoluteEncoderOffset = Preferences.getDouble(modulePosition.name() + absEncoderOffsetKey, defaultAbsoluteEncoderOffset);
        kPTurning = Preferences.getDouble(modulePosition.name() + kPTurningKey, defaultkPTurning);
        kITurning = Preferences.getDouble(modulePosition.name() + kITurningKey, defaultkITurning);
        kDTurning = Preferences.getDouble(modulePosition.name() + kDTurningKey, defaultkDTurning);
    }

    /**
     * Update the absolute encoder and drive encoder entries for this module in NetworkTables.
     */
    private void updateNetworkTables() {
        absoluteEncoderEntry.setDouble(getAbsoluteTurningAngle().getDegrees());
        driveEncoderEntry.setValue(driveEncoder);
    }

    /**
     * Returns the absolute angle of the module in radians.
     * @return The absolute encoder angle, CCW positive relative to robot front
     */
    public Rotation2d getAbsoluteTurningAngle(){
        double angle = absoluteEncoder.getVoltage()/RobotController.getVoltage5V();
        angle *= 2.0*Math.PI;
        angle -= absoluteEncoderOffset;
        if(absoluteEncoderInverted)
            angle *= -1.0;
        return new Rotation2d(angle);
    }

    /**
     * Resets the absolute encoder reading to zero.
     * <p>
     * WARNING: USE WITH CAUTION! This method should only be used when the module is facing forward and ready to zero, previous encoder offset will be erased.
     */
    public void rezeroAbsoluteEncoder() {
        Preferences.setDouble(absEncoderOffsetKey, absoluteEncoder.getVoltage()/RobotController.getVoltage5V() * 2.0 * Math.PI);
    }

    /**
     * Returns the position of the turning encoder in radians.
     * <p>
     * NOTE: use {@link #getAbsoluteTurningAngle()} instead, motor rotations are unreliable
     * @return steering encoder position in radians
     */
    public Rotation2d getTurningAngle() {
        return new Rotation2d(turningEncoder.getPosition());
    }

    /**
     * Returns the velocity of the turning encoder in radians per second.
     * @return turning encoder velocity in radians per second
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * Returns the position of the drive encoder in meters
     * @return drive encoder position in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the velocity of the drive encoder in meters per second.
     * @return drive encoder velocity in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Resets the distance reading of the drive encoder to 0, and the angle reading of the turning encoder to the absolute encoder's angle.
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteTurningAngle().getRadians());
    }

    /**
     * Get the current state of the module.
     * @return The state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurningAngle());
    }

    /**
     * Get the current position of the module.
     * @return The position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAbsoluteTurningAngle());
    }

    /**
     * Set the desired state of the module.
     * @
     * @param state The desired state of the module.
     */
    public void setDesiredState(SwerveModuleState state) {
        if (isLocked) {
            state = new SwerveModuleState(0, modulePosition.getLockRotation());
        } else if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND);
        turningMotor.set(turningPidController.calculate(getAbsoluteTurningAngle().getRadians(), state.angle.getRadians()));
        updateNetworkTables();
    }

    /**
     * Stop the module.
     */
    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    /**
     * Toggles the lock state of the module.
     * @return New lock/unlock state
     */
    public boolean toggleLocked() {
        isLocked = !isLocked;
        return isLocked;
    }

    /**
     * Command to rezero the absolute encoder.
     * @return InstantCommand to rezero the absolute encoder
     */
    public InstantCommand rezeroCommand() {
        return new InstantCommand(() -> this.rezeroAbsoluteEncoder());
    }
}