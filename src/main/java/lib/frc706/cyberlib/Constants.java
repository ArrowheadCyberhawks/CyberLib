package lib.frc706.cyberlib;

public final class Constants {
	public static final class SwerveModule {
		public static final String absEncoderOffsetKey = "absoluteEncoderOffset";
		public static final String kPTurningKey = "kPTurning";
		public static final String kITurningKey = "kITurning";
		public static final String kDTurningKey = "kDTurning";

		public static final double defaultAbsoluteEncoderOffset = 0.0;
		public static final double defaultkPTurning = 1.0;
		public static final double defaultkITurning = 0.0;
		public static final double defaultkDTurning = 0.1;
	}

	public static final class SparkPID {
		public static final double defaultkP = 1.0;
		public static final double defaultkI = 0.0;
		public static final double defaultkD = 0.0;
		public static final double defaultkFF = 1.0;
		public static final double defaultkIz = 0.0;
	}
}
