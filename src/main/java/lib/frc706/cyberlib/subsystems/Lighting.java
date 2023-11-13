package lib.frc706.cyberlib.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private static DigitalOutput out0, out1, out2, out3;
    
    public void robotInit() {
        out0 = new DigitalOutput(0);
        out1 = new DigitalOutput(1);
        out2 = new DigitalOutput(2);
        out3 = new DigitalOutput(3);
        setLEDPins(false, false, false, false);
        chooser.setDefaultOption("Pink", "PINK");
		chooser.addOption("Black", "BLACK");
		chooser.addOption("Yellow", "YELLOW");
		chooser.addOption("Purple", "PURPLE");
		chooser.addOption("Red", "RED");
		chooser.addOption("Blue", "BLUE");
    }
    
    public SendableChooser<String> getChooser() {
        return chooser;
    }
    public static void setLEDS(String color) {
        switch (color) {
            case "BLACK":
                setLEDPins(false, false, false, false);
                break;
            case "PINK":
                setLEDPins(true, true, true, true);
                break;
            case "YELLOW":
                setLEDPins(true, false, false, false);
                break;
            case "PURPLE":
                setLEDPins(false, true, false, false);
                break;
            case "RED":
                setLEDPins(false, false, true, false);
                break;
            case "BLUE":
                setLEDPins(false, false, false, true);
                break;
            default:
                break;
        }
    }
    public static void setLEDPins(boolean pin0, boolean pin1, boolean pin2, boolean pin3) {
        out0.set(pin0);
        out1.set(pin1);
        out2.set(pin2);
        out3.set(pin3);
    }
}
