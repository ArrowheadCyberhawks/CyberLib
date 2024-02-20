package lib.frc706.cyberlib;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class BrushlessSparkWithPID {

    
    public static double NEO550_MAXRPM = 11000.0;
    public static double NEO1650_MAXRPM = 5676.0;

    public CANSparkMax spark;
    SparkPIDController PIDController;
    public RelativeEncoder encoder;    
    private int sparkID;
    private int smartMotionSlot = 0;
    private int pidSlot = 0;

    private double motorPos = 0;
    private double motorVel = 0;
    private double positionOffset = 0;

    /**
     * Creates a new BrushlessSparkWithPID for simple control of a brushless motor
     * @param sparkID can id of the spark
     */
    public BrushlessSparkWithPID(int sparkID) {
        this.sparkID = sparkID;
        spark = new CANSparkMax(sparkID, MotorType.kBrushless); //create the spark
        PIDController = spark.getPIDController();         //initializing PID controller on Spark1(for sending inputs)
        encoder = spark.getEncoder();
    }
    
    /**
     * Creates a new BrushlessSparkWithPID for control of a brushless motor with PID with specified constants
     * @param sparkID can id of the spark
     * @param kP proportional constant
     * @param kI integral constant
     * @param kD derivative constant
     * @param kFF feed forward constant
     * @param kIz integral zone
     * @param maxVel max velocity of the motor in RPM
     * @param maxAcc max acceleration of the motor in RPM per second
     * @param allowedErr allowed error of the motor in rotations
     */
    public BrushlessSparkWithPID(int sparkID,  double kP, double kI, double kD, double kFF, double kIz, double maxVel, double maxAcc, double allowedErr){
        
        this.sparkID = sparkID;
        spark = new CANSparkMax(sparkID, MotorType.kBrushless); //create the spark
        spark.restoreFactoryDefaults();
        PIDController = spark.getPIDController();         //initializing PID controller on Spark1(for sending inputs)
        encoder = spark.getEncoder();                     //initializing Encoder on Spark1(to get state of spark1)
        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setFF(kFF);
        PIDController.setIZone(kIz);
        PIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);//spark ID is used as the smart motion id
    
    }

    public void updateConstants(double kP, double kI, double kD, double kFF, double kIz, double maxVel, double maxAcc, double allowedErr){
        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setFF(kFF);
        PIDController.setIZone(kIz);
        PIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);//spark ID is used as the smart motion id
    }

    public void updateSensorValues(){
        motorPos = encoder.getPosition()-positionOffset;
        motorVel = encoder.getVelocity();
    }

    public double getVelocity() {
        updateSensorValues();
        return motorVel;
    }

    public double getPosition(){
        updateSensorValues();
        return motorPos;
    }

    public void setPower(double value){
        spark.set(value);//set the power of the spark
    }

    public void setPos(double value){
        PIDController.setReference(value+positionOffset, ControlType.kPosition, pidSlot);//set the position of the spark
        System.out.println(value+positionOffset + " " + pidSlot);
    }

    public void setVel(double value){
        PIDController.setReference(value, ControlType.kVelocity, pidSlot);//set the velocity of the spark
    }

    public void setPIDSlot(int slot) {
        this.pidSlot = slot;
    }

    public double getRawOutput(){
        return spark.getAppliedOutput();//read the power the spark is trying to write
    }

    public void rezero(){
        positionOffset = encoder.getPosition();
    }

    public void rezero(double currentPos){
        positionOffset = encoder.getPosition()-currentPos;
    }

    public void setConversionFactors(double positionConversionFactor, double velocityConversionFactor){
        encoder.setPositionConversionFactor(positionConversionFactor);
        encoder.setVelocityConversionFactor(velocityConversionFactor);
    }
}