package lib.frc706.cyberlib;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static lib.frc706.cyberlib.Constants.SparkPID.*;

public class BrushlessSparkWithPID {

    
    public static double NEO550_MAXRPM = 11000.0;
    public static double NEO1650_MAXRPM = 5676.0;

    public CANSparkMax spark;
    SparkPIDController PIDController;
    public RelativeEncoder encoder;    
    int sparkID;
    int smartMotionSlot = 0;

    public double motorPos = 0;
    public double motorVel = 0;
    public double positionOffset = 0;

    /**
     * Creates a new BrushlessSparkWithPID for simple control of a brushless motor
     * @param sparkID can id of the spark
     */
    public BrushlessSparkWithPID(int sparkID) {
        this.sparkID = sparkID;
        spark = new CANSparkMax(sparkID, MotorType.kBrushless); //create the spark
        spark.restoreFactoryDefaults();
        PIDController = spark.getPIDController();         //initializing PID controller on Spark1(for sending inputs)
        encoder = spark.getEncoder();                     //initializing Encoder on Spark1(to get state of spark1)
        PIDController.setI(defaultkI);
        PIDController.setD(defaultkD);
        PIDController.setP(defaultkP);
        PIDController.setFF(defaultkFF);
        PIDController.setIZone(defaultkIz);
        PIDController.setSmartMotionMaxVelocity(NEO1650_MAXRPM, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionMaxAccel(NEO1650_MAXRPM, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);//spark ID is used as the smart motion id
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

    public double getPos(){
        updateSensorValues();
        return motorPos;
    }

    public void setPower(double value){
        spark.set(value);//set the power of the spark
    }

    public void setPos(double value){
        PIDController.setReference(value+positionOffset, ControlType.kSmartMotion);//set the position of the spark
    }

    public void setVel(double value){
        PIDController.setReference(value, ControlType.kSmartVelocity);//set the velocity of the spark
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
}