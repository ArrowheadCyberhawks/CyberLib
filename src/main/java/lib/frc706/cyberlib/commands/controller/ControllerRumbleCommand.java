package lib.frc706.cyberlib.commands.controller;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ControllerRumbleCommand extends Command {
    private XboxController controller;
    private BooleanSupplier condition;
    private double power;

    public ControllerRumbleCommand(XboxController controller, BooleanSupplier condition, double power) {
        this.controller = controller;
        this.condition = condition;
        this.power = power;
    }

    public ControllerRumbleCommand(XboxController controller, double power) {
        this.controller = controller;
        this.power = power;
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean()) {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, power);
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
}
