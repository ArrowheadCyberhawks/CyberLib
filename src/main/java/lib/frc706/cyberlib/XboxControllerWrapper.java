package lib.frc706.cyberlib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * XboxControllerWrapper is a custom wrapper for the CommandXboxController that provides additional functionality
 * such as deadband processing and axis interpolation.
 */
public class XboxControllerWrapper extends CommandXboxController {
    private double kDeadband;
    private double interpolateMin = 0.15;

    /**
     * Constructs an XboxControllerWrapper with the specified port, deadband, and interpolation minimum.
     *
     * @param port the port number of the Xbox controller
     * @param kDeadband the deadband value to apply to the controller axes
     * @param interpolateMin the minimum value for interpolation
     */
    public XboxControllerWrapper(int port, double kDeadband, double interpolateMin) {
        super(port);
        this.kDeadband = kDeadband;
        this.interpolateMin = interpolateMin;
    }

    /**
     * Constructs an XboxControllerWrapper with the specified port and deadband.
     *
     * @param port the port number of the Xbox controller
     * @param kDeadband the deadband value to apply to the controller axes
     */
    public XboxControllerWrapper(int port, double kDeadband) {
        super(port);
        this.kDeadband = kDeadband;
    }

    /**
     * Processes the given axis value by applying deadband and squaring the value while preserving its sign.
     *
     * @param value the raw axis value
     * @return the processed axis value
     */
    public double processAxis(double value) {
        value = -value;
        value = MathUtil.applyDeadband(value, kDeadband);
        value = Math.copySign(value * value, value);
        return value;
    }

    /**
     * Interpolates the given value based on the right trigger axis.
     *
     * @param value the value to interpolate
     * @return the interpolated value
     */
    public double interpolate(double value) {
        return value * MathUtil.interpolate(interpolateMin, 1, getRightTriggerAxis());
    }

    /**
     * Gets the interpolated left Y axis value.
     *
     * @return the interpolated left Y axis value
     */
    @Override
    public double getLeftY() {
        return interpolate(processAxis(super.getLeftY()));
    }

    /**
     * Gets the interpolated left X axis value.
     *
     * @return the interpolated left X axis value
     */
    @Override
    public double getLeftX() {
        return interpolate(processAxis(super.getLeftX()));
    }

    /**
     * Gets the interpolated right Y axis value.
     *
     * @return the interpolated right Y axis value
     */
    @Override
    public double getRightY() {
        return interpolate(processAxis(super.getRightY()));
    }

    /**
     * Gets the interpolated right X axis value.
     *
     * @return the interpolated right X axis value
     */
    @Override
    public double getRightX() {
        return interpolate(processAxis(super.getRightX()));
    }

    /**
     * Gets the uninterpolated left Y axis value.
     *
     * @return the uninterpolated left Y axis value
     */
    public double getUninterpolatedLeftY() {
        return processAxis(super.getLeftY());
    }

    /**
     * Gets the uninterpolated left X axis value.
     *
     * @return the uninterpolated left X axis value
     */
    public double getUninterpolatedLeftX() {
        return processAxis(super.getLeftX());
    }

    /**
     * Gets the uninterpolated right Y axis value.
     *
     * @return the uninterpolated right Y axis value
     */
    public double getUninterpolatedRightY() {
        return processAxis(super.getRightY());
    }

    /**
     * Gets the uninterpolated right X axis value.
     *
     * @return the uninterpolated right X axis value
     */
    public double getUninterpolatedRightX() {
        return processAxis(super.getRightX());
    }

    /**
     * Gets the raw left Y axis value without any processing.
     *
     * @return the raw left Y axis value
     */
    public double getRawLeftY() {
        return super.getLeftY();
    }

    /**
     * Gets the raw left X axis value without any processing.
     *
     * @return the raw left X axis value
     */
    public double getRawLeftX() {
        return super.getLeftX();
    }

    /**
     * Gets the raw right Y axis value without any processing.
     *
     * @return the raw right Y axis value
     */
    public double getRawRightY() {
        return super.getRightY();
    }

    /**
     * Gets the raw right X axis value without any processing.
     *
     * @return the raw right X axis value
     */
    public double getRawRightX() {
        return super.getRightX();
    }
}