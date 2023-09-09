package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class TeamRobot 
{
    private static int CRANE_MAXIMUM_HEIGHT = 4270;
    int craneLiftMotor_minPositionSeen;
    boolean craneOverride=false;
    private static double CRANE_MOVEMENT_SPEED_UP = 0.7;
    private static double CRANE_MOVEMENT_SPEED_DOWN = 0.5;
    private static double CRANE_SERVO_GRAB = 0.75;
    private static double CRANE_SERVO_RELEASE = 0.90;
    
    private Blinker expansion_Hub_1;
    //private Gyroscope imu;
    private DcMotor craneLiftMotor;
    private DcMotor craneLowerMoter;
    private DcMotor m0;
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor driveMotors[];
    
    private Servo craneServo;
   
    double forwardPowers[] = new double[] {-1.0, -1.0, +1.0, +1.0};
    double backwardPowers[] = new double[] {+1.0, +1.0, -1.0, -1.0};
    double leftPowers[] = new double[] {-1.0, +1.0, -1.0, +1.0};
    double rightPowers[] = new double[] {+1.0, -1.0, +1.0, -1.0};
    
    // Gyro variables
    BNO055IMU imu;
    
    OpMode opMode;


    public TeamRobot(OpMode opMode)
    {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        m0 = hardwareMap.dcMotor.get("m0");
        m1 = hardwareMap.dcMotor.get("m1");
        m1.setDirection(DcMotorSimple.Direction.REVERSE);

        m2 = hardwareMap.dcMotor.get("m2");
        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        m3 = hardwareMap.dcMotor.get("m3");
       
        m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        driveMotors = new DcMotor[] {m0, m1, m2, m3};

        craneLiftMotor = hardwareMap.dcMotor.get("crane_lift");
        craneLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneLiftMotor_minPositionSeen = craneLiftMotor.getCurrentPosition();
        
        craneLowerMoter = hardwareMap.dcMotor.get("crane_lower");
        craneLowerMoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        craneServo = hardwareMap.servo.get("Crane_Servo");
        craneServo.setPosition(CRANE_SERVO_RELEASE);
       
        opMode.telemetry.addData("Crane:", "%s", this::getCraneTelemetry);

        // Initialize IMU/Gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        _getAngle1FromImu();
      //  opMode.telemetry.addData("IMU: ", "%s", this::getImuTelemetry);
    }


    public void loop()
    {
        opMode.telemetry.update();
        
        if (craneLiftMotor.getCurrentPosition() < craneLiftMotor_minPositionSeen)
            craneLiftMotor_minPositionSeen = craneLiftMotor.getCurrentPosition();
            
        if ( ! craneOverride ) {
            if (craneLiftMotor.getPower() <= 0 && craneLiftMotor.getCurrentPosition() <= craneLiftMotor_minPositionSeen) {
                craneStop();
            }
            
            if (craneLiftMotor.getPower() >= 0 && craneLiftMotor.getCurrentPosition() >= craneLiftMotor_minPositionSeen+CRANE_MAXIMUM_HEIGHT) {
                craneStop();
            }
        }        
    }
    
    public void craneUp()
    {
        craneLiftMotor.setPower(CRANE_MOVEMENT_SPEED_UP);
        craneLowerMoter.setPower(0.2);
        
    }
    
    public void craneDown(boolean overide)
    {
        craneOverride=overide;
        
        if (overide == true) {
            craneLiftMotor.setPower(-0.1);
            craneLowerMoter.setPower(-CRANE_MOVEMENT_SPEED_DOWN/2);
            return;
        }
        
        craneLiftMotor.setPower(-0.2);
        craneLowerMoter.setPower(-CRANE_MOVEMENT_SPEED_DOWN);
        
    }
    
    public void craneStop()
    {
        craneLiftMotor.setPower(0);
        craneLowerMoter.setPower(0);
    }
    
    public void craneServoGrab()
    {
        craneServo.setPosition(CRANE_SERVO_GRAB);
        
    }
    
    public void craneServoRelease()
    {
        craneServo.setPosition(CRANE_SERVO_RELEASE);
        
    }
    
    //USE : returns list of unscaled moter powers (must in put quadrant)
    //GOAL : Shouldn't consider the quadrant
    public double[] getPowerRatiosInQuadrant(double angleIntoQuadrant, double zeroDegPowers[], double ninetyDegPowers[])
    {
        double zeroFraction = (90.0 - angleIntoQuadrant) / 90.0;
        double ninetyFraction = angleIntoQuadrant / 90.0;
       
        double result[] = new double[4];
        for (int motorNumber = 0; motorNumber < 4; motorNumber++)
        {
            result[motorNumber] = zeroDegPowers[motorNumber] * zeroFraction + ninetyDegPowers[motorNumber] * ninetyFraction;
            //result[motorNumber] = 1.0 * (45/90) + -1.0 * (45/90)
        }
        return result;
    }
    
    //USE : returns a list of unscaled moter powers (works for all quadrants)
    //GOAL : Sould scaled moter powers and should consider quadrant
    public double[] getMotorPowersForDirection(double angle)
    {
        if (angle <= 90)
        {
            return getPowerRatiosInQuadrant(angle, rightPowers, forwardPowers);
        }
        else if (angle <= 180)
        {
            return getPowerRatiosInQuadrant(angle - 90, forwardPowers, leftPowers);
        }
        else if (angle <= 270)
        {
            return getPowerRatiosInQuadrant(angle - 180, leftPowers, backwardPowers);
        }
        else
        {
            return getPowerRatiosInQuadrant(angle - 270, backwardPowers, rightPowers);
        }
    }
    
    //USE : applys moter powers and scales them
    //GOAL : Shouldn't scale moter powers
    void setMotorPowers(double speedScale, double directionPowers[], double spinPowers[])
    {
        if (spinPowers == null)
            spinPowers = new double[]  {0.0, 0.0, 0.0, 0.0};
        if (directionPowers == null)
            directionPowers = new double[] {0.0, 0.0, 0.0, 0.0};
            
        double max = 0.0;
        double tempMotorPowers[] = {0.0, 0.0, 0.0, 0.0};
        for(int i = 0; i < 4; i++)
        {
            tempMotorPowers[i] = (directionPowers[i] * speedScale + spinPowers[i]);
            max = Math.max(max, Math.abs(tempMotorPowers[i]));
        }
        
        if (max > 1)
        {
            for(int i = 0; i < 4; i++)
            {
                tempMotorPowers[i] = tempMotorPowers[i] / max;
            }
        }
        
        for(int i = 0; i < 4; i++)
        {
            driveMotors[i].setPower(tempMotorPowers[i]);
        }
    }
    
    public String getCraneTelemetry() {
        return String.format("Pow: up%4.2f@%4d down:%4.2f@%4d| Claw:%s (%.2f)",
            craneLiftMotor.getPower(),
            craneLiftMotor.getCurrentPosition(),
            craneLowerMoter.getPower(),
            craneLowerMoter.getCurrentPosition(),
            craneServo.getPosition()==CRANE_SERVO_GRAB ? "Grab" : "Release",
            craneServo.getPosition());
    }
    
    public void moveForTime(double angle, long msec){
        long timeStarted = System.currentTimeMillis();
            
            long endTime = timeStarted + msec;
            
            while (System.currentTimeMillis()<endTime) {
                double motorPowers[] = getMotorPowersForDirection(angle);
                setMotorPowers(0.5, motorPowers, null);
                loop();
            }
            
    }
    
    private float _getAngle1FromImu() {
        return imu.getAngularOrientation().firstAngle;
    }

    private float _getAngle2FromImu() {
        return imu.getAngularOrientation().secondAngle;
    }
    private float _getAngle3FromImu() {
        return imu.getAngularOrientation().thirdAngle;
    }
    
    public String getImuTelemetry() {
        return String.format("A1=%.2f | A2=%.2f | A3=%.2f",
            _getAngle1FromImu(),
            _getAngle2FromImu(),
            _getAngle3FromImu());
    }

}
