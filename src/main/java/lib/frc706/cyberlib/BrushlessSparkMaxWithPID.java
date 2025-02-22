package lib.frc706.cyberlib;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

public class BrushlessSparkMaxWithPID {

    
    public static double NEO550_MAXRPM = 11000.0;
    public static double NEO1650_MAXRPM = 5676.0;

    public SparkMax spark;
    public SparkClosedLoopController closedLoopController;
    ClosedLoopConfigAccessor closedLoopControllerAccessor;
    private SparkMaxConfig config;
    public RelativeEncoder encoder;

    private double motorPos = 0;
    private double motorVel = 0;
    private double positionOffset = 0;

    /**
     * Creates a new BrushlessSparkWithPID for simple control of a brushless motor
     * @param sparkID can id of the spark
     */
    public BrushlessSparkMaxWithPID(int sparkID) {
        spark = new SparkMax(sparkID, MotorType.kBrushless); //create the spark
        closedLoopController = spark.getClosedLoopController();         //initializing PID controller on Spark1(for sending inputs)
        closedLoopControllerAccessor = spark.configAccessor.closedLoop;         //initializing PID controller on Spark1(for sending inputs)
        encoder = spark.getEncoder();
        config = new SparkMaxConfig();
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
    public BrushlessSparkMaxWithPID(int sparkID,  double kP, double kI, double kD, double kFF, double kIz, double maxVel, double maxAcc, double allowedErr){
        
        spark = new SparkMax(sparkID, MotorType.kBrushless); //create the spark
        closedLoopController = spark.getClosedLoopController();        //initializing PID controller on Spark1(for sending inputs)
        encoder = spark.getEncoder();                     //initializing Encoder on Spark1(to get state of spark1)
        config = new SparkMaxConfig();
        config.closedLoop.pidf(kP, kI, kD, kFF);
        config.closedLoop.iZone(kIz);
        config.closedLoop.maxMotion.maxVelocity(maxVel);//spark ID is used as the smart motion id
        config.closedLoop.maxMotion.maxAcceleration(maxAcc);//spark ID is used as the smart motion id
        config.closedLoop.maxMotion.allowedClosedLoopError(allowedErr);//spark ID is used as the smart motion id
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateConstants(double kP, double kI, double kD, double kFF, double kIz, double maxVel, double maxAcc, double allowedErr){
        config.closedLoop.pidf(kP, kI, kD, kFF);
        config.closedLoop.iZone(kIz);
        config.closedLoop.maxMotion.maxVelocity(maxVel);//spark ID is used as the smart motion id
        config.closedLoop.maxMotion.maxAcceleration(maxAcc);//spark ID is used as the smart motion id
        config.closedLoop.maxMotion.allowedClosedLoopError(allowedErr);//spark ID is used as the smart motion id 
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        closedLoopController.setReference(value+positionOffset, ControlType.kPosition);//set the position of the spark
    }

    public void setVel(double value){
        closedLoopController.setReference(value, ControlType.kVelocity);//set the velocity of the spark
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
        config.encoder.positionConversionFactor(positionConversionFactor);
        config.encoder.velocityConversionFactor(velocityConversionFactor);
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}