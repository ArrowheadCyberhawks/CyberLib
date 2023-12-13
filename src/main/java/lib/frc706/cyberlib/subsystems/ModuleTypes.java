package lib.frc706.cyberlib.subsystems;

/*
 * Another class stolen directly from swervelib.
 */
public final class ModuleTypes {
    public static final ModuleType MK3_STANDARD = new ModuleType(
            0.1016,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final ModuleType MK3_FAST = new ModuleType(
            0.1016,
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );

    public static final ModuleType MK4_L1 = new ModuleType(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final ModuleType MK4_L2 = new ModuleType(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final ModuleType MK4_L3 = new ModuleType(
            0.10033,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final ModuleType MK4_L4 = new ModuleType(
            0.10033,
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );

    public static final ModuleType MK4I_L1 = new ModuleType(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    );
    public static final ModuleType MK4I_L2 = new ModuleType(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    );
    public static final ModuleType MK4I_L3 = new ModuleType(
            0.10033,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    );
}
