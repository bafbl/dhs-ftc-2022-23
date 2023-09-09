package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.ArrayList;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class Experimental_TeamRobot 
{
    private static int CRANE_MAXIMUM_HEIGHT = 4356;
    private static double CRANE_MOVEMENT_SPEED_UP = 0.7;
    private static double CRANE_OPPOSITE_SPEED_UP = 0.4;
    private static double CRANE_MOVEMENT_SPEED_DOWN = 0.7;
    private static double CRANE_OPPOSITE_SPEED_DOWN = 0.35;
    
    private static double CRANE_SERVO_GRAB = 0.75;
    private static double CRANE_SERVO_RELEASE = 0.90;
    private Servo craneServo;
    private DcMotor craneLiftMotor;
    private DcMotor craneLowerMoter;
    private TouchSensor craneTouchSensor;
    int craneLiftMotor_minPositionSeen;
    boolean craneOverride=false;

    private DcMotor m0;
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor driveMotors[];

    private List<String> telemetryMessages_list = new ArrayList<>();    
    private String telemetryMessages_string="";
   
    double forwardPowers[] = new double[] {-1.0, -1.0, +1.0, +1.0};
    double backwardPowers[] = new double[] {+1.0, +1.0, -1.0, -1.0};
    double leftPowers[] = new double[] {-1.0, +1.0, -1.0, +1.0};
    double rightPowers[] = new double[] {+1.0, -1.0, +1.0, -1.0};
    
    // Gyro variables
    BNO055IMU imu;
    
    // Forward is zero, 360, etc
    double totalDegreesTurned = 0;
    Double desiredTotalDegreesTurned = null;

    // Remember the previous direction so we know how much we turned
    double previousDirection = 0;


    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    final float[] hsvValues = new float[3];
    
    LinearOpMode opMode;


    public Experimental_TeamRobot(LinearOpMode opMode)
    {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        m0 = hardwareMap.dcMotor.get("m0");
        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
       
        m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        driveMotors = new DcMotor[] {m0, m1, m2, m3};

        craneLiftMotor = hardwareMap.dcMotor.get("crane_lift");
        craneLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneLiftMotor_minPositionSeen = craneLiftMotor.getCurrentPosition();
        craneLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        craneLowerMoter = hardwareMap.dcMotor.get("crane_lower");
        craneLowerMoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        craneServo = hardwareMap.servo.get("Crane_Servo");
        craneServo.setPosition(CRANE_SERVO_RELEASE);
        craneTouchSensor = hardwareMap.get(TouchSensor.class, "crane_touch");
       
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        //colorSensor.enableLed(false);

        opMode.telemetry.addData("Drive", "%s", this::getDriveTelemetry);
        opMode.telemetry.addData("Color", "%s", this::getColorTelemetry);
        
        opMode.telemetry.addData("Crane", "%s", this::getCraneTelemetry);

        // Initialize IMU/Gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        _getAngle1FromImu();
        opMode.telemetry.addData("IMU", "%s", this::getImuTelemetry);
        
        opMode.telemetry.addData("Messages", "\n%s", this::getTelemetryMessages);
    }

    public boolean shouldWeKeepRunning() {
        if (!opMode.opModeIsActive())
            return false;
        else
            return true;
    }

    public void loop(String telemetryMessageFmt, Object... args) {
        addTelemetryMessage(telemetryMessageFmt, args);
        loop();
    }
    
    
    public void loop()
    {
        if (craneTouchSensor.isPressed() 
            || craneLiftMotor.getCurrentPosition() < craneLiftMotor_minPositionSeen)
        {
            craneSetCurrentPositionAsMinimum();
        }
            
        if ( ! craneOverride ) {
            if (craneLiftMotor.getPower() <= 0 && !canCraneGoDown())
                craneStop();
            
            if (craneLiftMotor.getPower() >= 0 && !canCraneGoUp())
                craneStop();
        }       
        
        double currentDirection = _getAngle1FromImu();
        // TODO:
        // ... use newDirection and previousDirection and wrap-around
        // to update totalDegreesTurned
        double degChange = currentDirection - previousDirection;
        if(degChange > 180)
            degChange = degChange - 360;
        else if(degChange < -180)
            degChange = degChange + 360;
        totalDegreesTurned = totalDegreesTurned + degChange;
        previousDirection = currentDirection;

        // Assemble telemetry messages
        StringBuilder telemetryMessages_sb=new StringBuilder();
        for (String message : telemetryMessages_list) {
            if ( telemetryMessages_sb.length() != 0 )
              telemetryMessages_sb.append("\n");
            telemetryMessages_sb.append(message);
        }
        telemetryMessages_string = telemetryMessages_sb.toString();
        telemetryMessages_list.clear();

        opMode.telemetry.update();
    }
    
    public String getColorTelemetry() {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return String.format("\nRed: %4.3f Gre: %4.3f Blu: %4.3f \nHue: %4.1f Sat: %4.3f Val: %4.3f \nAlf: %4.3f\nZone: %d", 
            colors.red, colors.green, colors.blue, hsvValues[0], hsvValues[1], hsvValues[2], colors.alpha, findParkingZone());
    }
    
    public String getTelemetryMessages() {
        return telemetryMessages_string;
    }
    
    public void addTelemetryMessage(String fmt, Object... args) {
        try {
            telemetryMessages_list.add(String.format(fmt, args));
        } catch (Exception e) {
            telemetryMessages_list.add(String.format("Bad format: %s: %s", fmt, e.getMessage()));
        }
    }
    
    public int findParkingZone() {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        double red = colors.red, blue = colors.blue, green = colors.green;
        if (red + blue + green > 2) 
            return 3;
        if (red > blue) 
            return 2;
        return 1;
    }
    
    public void craneSetCurrentPositionAsMinimum() {
        craneLiftMotor_minPositionSeen = craneLiftMotor.getCurrentPosition();
    }
    
    public int getCraneHeight() {
        return craneLiftMotor.getCurrentPosition() - craneLiftMotor_minPositionSeen;
    }
    
    public boolean canCraneGoDown() {
        if (craneTouchSensor.isPressed())
            return false;
        return getCraneHeight() > 0;
    }
    
    public boolean canCraneGoUp() {
        return getCraneHeight() < CRANE_MAXIMUM_HEIGHT;
    }
    
    public void setCraneOverride(boolean override) {
        craneOverride=override;
    }
    
    public void craneUp()
    {
        if (craneOverride == true) {
            craneLiftMotor.setPower(CRANE_MOVEMENT_SPEED_UP/2);
            craneLowerMoter.setPower(CRANE_OPPOSITE_SPEED_UP/2);
            return;
        }
        
        if (!canCraneGoUp()) {
            craneStop();
            return;
        }
            
        craneLiftMotor.setPower(CRANE_MOVEMENT_SPEED_UP);
        craneLowerMoter.setPower(CRANE_OPPOSITE_SPEED_UP);
    }
    
    public void craneDown()
    {
        // Always stop if the touch sensor is pressed. Not even override can
        // go lower
        if (craneTouchSensor.isPressed()) {
            craneStop();
            return;
        }
            
        if (craneOverride == true) {
            craneLiftMotor.setPower(-CRANE_OPPOSITE_SPEED_DOWN/2);
            craneLowerMoter.setPower(-CRANE_MOVEMENT_SPEED_DOWN/2);
            return;
        }

        if (!canCraneGoDown()) {
            craneStop();
            return;
        }
            
        craneLiftMotor.setPower(-CRANE_OPPOSITE_SPEED_DOWN);
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
    
    public void movecrantoposition(int height_inches)
    {
        int height_ticks = (int) (185.5 * height_inches) ; 
        if(height_ticks == 0){
            craneDown();
            while(shouldWeKeepRunning() && craneTouchSensor.isPressed()==false)
                loop("Lowering crane until button is pressed");
            craneStop();
        }
        else if(getCraneHeight() >= height_ticks){
            craneDown();
            while(shouldWeKeepRunning() && getCraneHeight() > height_ticks)
                loop("Lowering crane. %d ticks to go", getCraneHeight()-height_ticks);
            craneStop();
        } else {
            craneUp();
            while(shouldWeKeepRunning() && getCraneHeight() <= height_ticks)
                loop("Raising crane. %d ticks to go", height_ticks-getCraneHeight());
            craneStop();
        }
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
    
    //USE : applies moter powers and scales them
    //GOAL : Shouldn't scale motor powers
    void setMotorPowers(double speedScale, double directionPowers[], double spinPowers[])
    {
        if (spinPowers == null)
            spinPowers = new double[]  {0.0, 0.0, 0.0, 0.0};
        if (directionPowers == null)
            directionPowers = new double[] {0.0, 0.0, 0.0, 0.0};
            
        if(spinPowers[0] != 0.0){
            // If a spin power is specified, then clear the desired heading
            desiredTotalDegreesTurned=null;
        } else {
            // if no spin was specified by caller, see if spin is needed to achieve desired heading
            if(desiredTotalDegreesTurned != null){
                double spinDirectionForHeadingCorrection;
                double spinPowerForHeadingCorrection;
                double headingDifference = desiredTotalDegreesTurned - totalDegreesTurned;
                if ( headingDifference>0 )
                    // counter clockwise
                    spinDirectionForHeadingCorrection=+1;
                else
                    // clockwise
                    spinDirectionForHeadingCorrection=-1;

                // scale the spinPower based on how much we need to turn
                double headingErrorMagnitude=Math.abs(headingDifference);
                if (headingErrorMagnitude < 0.5)
                    // We're in the right spot
                    spinPowerForHeadingCorrection = 0; 
                else if ( headingErrorMagnitude > 30 ) 
                    //  we have more than 30 degrees to turn... full power
                    spinPowerForHeadingCorrection = 1.0;
                else {
                    // Scale correction linearly with a minimum
                    spinPowerForHeadingCorrection = 1.0*headingErrorMagnitude/30;
                    spinPowerForHeadingCorrection = Math.max(spinPowerForHeadingCorrection, 0.1);
                }
                
                double spin=spinDirectionForHeadingCorrection*spinPowerForHeadingCorrection;
                addTelemetryMessage("Heading correction: %+4.2f for %.1fdeg error",
                    spin, headingDifference);
                    
                spinPowers= new double[] {spin,spin,spin,spin};
            }
        }
        
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
    
    public String getDriveTelemetry() {
        return String.format("M0: %.2f@%5d, M1: %.2f@%5d, M2: %.2f@%5d, M3: %.2f@%5d",
            m0.getPower(), m0.getCurrentPosition(),
            m1.getPower(), m1.getCurrentPosition(),
            m2.getPower(), m2.getCurrentPosition(),
            m3.getPower(), m3.getCurrentPosition());
    }
    
    public String getCraneTelemetry() {
        return String.format("Pos:[%4d,%4d,%4d] h=%4d touch:%s|Pow: up%4.2f down:%4.2f override:%s|Claw:%s (%.2f)",
            craneLiftMotor_minPositionSeen,
            craneLiftMotor.getCurrentPosition(),
            craneLiftMotor_minPositionSeen+CRANE_MAXIMUM_HEIGHT,
            getCraneHeight(),
            craneTouchSensor.isPressed() ? "Pressed  " : "Unpressed",
            craneLiftMotor.getPower(),
            craneLowerMoter.getPower(),
            craneOverride ? "Y" : "N",
            craneServo.getPosition()==CRANE_SERVO_GRAB ? "Grab" : "Release",
            craneServo.getPosition());
    }
    
    public void moveForTime(double angle, long msec, double power){
        long timeStarted = System.currentTimeMillis();
            
            long endTime = timeStarted + msec;
            
            while (shouldWeKeepRunning() && System.currentTimeMillis()<endTime) {
                double motorPowers[] = getMotorPowersForDirection(angle);
                setMotorPowers(power, motorPowers, null);
                loop("Moving at %.0f degrees with pow=%.2f. %.1f secs to go",
                        angle, power, (1.0*endTime-System.currentTimeMillis())/1000);
            }
            
    }
    
    public void moveForTime(double angle, long msec){
        moveForTime(angle, msec, 0.5);
    }
    
    // counterclockwise is positive; clockwise is negative
    public void spin(double spinPower) {
        setMotorPowers(spinPower, null, new double[] {1,1,1,1});
    }
    
    public void turnForAngle(double turnAngle, double turnSpeed){
        if ( desiredTotalDegreesTurned == null )
            desiredTotalDegreesTurned = totalDegreesTurned + turnAngle;
        else
            desiredTotalDegreesTurned += turnAngle;
        
        while (shouldWeKeepRunning() && Math.abs(getHeadingError()) > 2 ) {
            // stop will only spin to correct the heading
            stop();
            loop("Turning %s dir. %.0f deg left", 
                turnAngle>0 ? "CW" : "CCW",
                Math.abs(getHeadingError()));
        }
    }
    
    // Set heading: 0--359.9
    public void setDesiredHeading(double desiredHeading){
        // This is based on totalDegreesTurned.
        
        // Find the closest number of times the robot has spun around
        double fullRotations = Math.round(totalDegreesTurned/360);

        // Because we're always adding the desiredHeading, 
        // this may include an extra 360-degree rotations        
        double possibleDesiredTotalDegreesTurned = 360.0*fullRotations + desiredHeading;
        
        // Remove extra 360 turns... we should always be within 180 degrees of 
        // the desired heading, so add/subtract a rotation if we're too far away
        if ( possibleDesiredTotalDegreesTurned-totalDegreesTurned > 180 )
            possibleDesiredTotalDegreesTurned -= 360;
        else if ( possibleDesiredTotalDegreesTurned-totalDegreesTurned < -180 )
            possibleDesiredTotalDegreesTurned += 360;
            
        desiredTotalDegreesTurned = possibleDesiredTotalDegreesTurned;
    }
    
    public double getHeadingError() {
        if (desiredTotalDegreesTurned == null)
            return 0;
        else
            return desiredTotalDegreesTurned - totalDegreesTurned;
    }
    
    // Stop the robot's movement other than correcting for heading errors
    public void stop() {
        setMotorPowers(0, null, null);
    }
    
    public void waitForTime(long msec){
        long timeStarted = System.currentTimeMillis();
            
            long endTime = timeStarted + msec;
            
            while (shouldWeKeepRunning() && System.currentTimeMillis()<endTime) {
                loop("Waiting for %.1fsec. %.1fsec left",
                    1.0*msec/1000, (1.0*endTime-System.currentTimeMillis())/1000);
            }
            
    }
    
    private double TICKS_PER_INCH=500;
    
    public void moveForDistance(double angle, double distanceInInches){
        int startPosition = m0.getCurrentPosition();

        double motorPowers[] = getMotorPowersForDirection(angle);
        setMotorPowers(0.5, motorPowers, null);

        int distanceInTicks = (int) (distanceInInches * TICKS_PER_INCH);
        
        int remainingTicks = Math.abs(m0.getCurrentPosition()-startPosition) - distanceInTicks;
        while (shouldWeKeepRunning() && remainingTicks>0) {
                loop("Moving for %.2finches (%d ticks) at %.0f degrees. %d ticks to go.",
                    distanceInInches, distanceInTicks, angle, remainingTicks );
                remainingTicks = Math.abs(m0.getCurrentPosition()-startPosition) - distanceInTicks;
        }
        setMotorPowers(0, null, null);
            
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
    
    // [0,360) where 0 is 'North', 90 is 'West', 180 is 'South' and 270 is 'East'
    // North is defined to be the direction the robot was facing when it started
    // or reset to
    public double getCurrentHeading() {
        double heading=totalDegreesTurned % 360;
        if ( heading>=0 )
            return heading;
        else
            return heading+360;
    }
    
    public void resetHeadingToClosestWaypoint() {
        double hdg = getCurrentHeading();
        
        // Heading is from 0 to 359.9
        if ( hdg <= 45 )
            resetHeading_North();
        else if ( hdg <= 135 )
            resetHeading_West();
        else if ( hdg <= 225 )
            resetHeading_South();
        else if ( hdg <= 315 )
            resetHeading_East();
        else
            resetHeading_North();
    }
    
    public void resetHeading_North() {
        totalDegreesTurned=0;
        desiredTotalDegreesTurned=null;
    }
    
    public void resetHeading_South() {
        totalDegreesTurned=180;
        desiredTotalDegreesTurned=null;
    }
    
    public void resetHeading_East() {
        totalDegreesTurned=270;
        desiredTotalDegreesTurned=null;
    }
    
    public void resetHeading_West() {
        totalDegreesTurned=90;
        desiredTotalDegreesTurned=null;
    }
    
    public String getImuTelemetry() {
        return String.format("TotalDeg=%+4.0f |Hdg: Cur=%3.0f|Goal=%s Err=%+3.0f|\nA1=%+.1f | A2=%+.1f | A3=%+.1f",
            totalDegreesTurned,
            getCurrentHeading(),
            desiredTotalDegreesTurned == null ? "none" : String.format("%4.0f",desiredTotalDegreesTurned),
            getHeadingError(),
            _getAngle1FromImu(),
            _getAngle2FromImu(),
            _getAngle3FromImu());
    }

    public double getRobotAngle() {
        return _getAngle1FromImu();
        
    }
}
