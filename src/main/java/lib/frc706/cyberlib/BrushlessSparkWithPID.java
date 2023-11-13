package lib.frc706.cyberlib;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class BrushlessSparkWithPID {

    public CANSparkMax spark;
    SparkMaxPIDController PIDController;
    public RelativeEncoder encoder;    
    int sparkID;
    int smartMotionSlot = 0;

    public double motorPos = 0;
    public double motorVel = 0;
    public double positionOffset = 0;

    public BrushlessSparkWithPID(int sparkID, double kP, double kI, double kD, double kFF, double kIz, double kMinOutput, double kMaxOutput, double maxVel, double minVel, double maxAcc, double allowedErr){
        
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
        PIDController.setOutputRange(kMinOutput, kMaxOutput);
        PIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);//spark ID is used as the smart motion id
    
    }

    public void updateConstants(double kP, double kI, double kD, double kFF, double kIz, double kMinOutput, double kMaxOutput, double maxVel, double minVel, double maxAcc, double allowedErr){
        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setFF(kFF);
        PIDController.setIZone(kIz);
        PIDController.setOutputRange(kMinOutput, kMaxOutput);
        PIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);//spark ID is used as the smart motion id
        PIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);//spark ID is used as the smart motion id
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
        PIDController.setReference(value+positionOffset, CANSparkMax.ControlType.kSmartMotion);//set the position of the spark
    }

    public void setVel(double value){
        PIDController.setReference(value, CANSparkMax.ControlType.kSmartVelocity);//set the velocity of the spark
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