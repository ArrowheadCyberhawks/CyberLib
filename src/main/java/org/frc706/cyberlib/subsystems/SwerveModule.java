package org.frc706.cyberlib.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;


public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;


    final AnalogInput absoluteEncoder;
    private final double absoluteEncoderOffsetRad;
    public final double initPos;

    final ModuleConfiguration moduleConfiguration;

    static double MAX_VELOCITY_METERS_PER_SECOND;

    public enum ModuleType {
        MK4_L1,
        MK4_L2,
        MK4_L3,
        MK4_L4
    }

    public SwerveModule(ModuleType moduleType, int driveMotorId, int turningMotorId, int absoluteEncoderId, double absoluteEncoderOffset, double kPTurning) {
        switch (moduleType) {
            case MK4_L1:
                moduleConfiguration = SdsModuleConfigurations.MK4_L1;
                break;
            case MK4_L2:
                moduleConfiguration = SdsModuleConfigurations.MK4_L2;
                break;
            case MK4_L3:
                moduleConfiguration = SdsModuleConfigurations.MK4_L3;
                break;
            case MK4_L4:
                moduleConfiguration = SdsModuleConfigurations.MK4_L4;
                break;
            default:
                throw new IllegalArgumentException("Invalid moduleType!");
        }
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(moduleConfiguration.getDriveReduction() * Math.PI * moduleConfiguration.getWheelDiameter());
        driveEncoder.setVelocityConversionFactor(moduleConfiguration.getDriveReduction() * Math.PI * moduleConfiguration.getWheelDiameter() / 60);
        turningEncoder.setPositionConversionFactor(moduleConfiguration.getSteerReduction() * 2 * Math.PI);
        turningEncoder.setVelocityConversionFactor(moduleConfiguration.getSteerReduction() * 2 * Math.PI / 60);
        MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * moduleConfiguration.getDriveReduction() * moduleConfiguration.getWheelDiameter()
            * Math.PI;
        turningPidController = new PIDController(kPTurning, 0, 0); 
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        initPos = getAbsoluteEncoderRad();
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage()/RobotController.getVoltage5V();
        angle *= 2.0*Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle;


    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        //state.angle = state.angle.plus(new Rotation2d(absoluteEncoderOffsetRad));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));

    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }
}