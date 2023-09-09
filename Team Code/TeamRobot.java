package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.*;
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

public class TeamRobot 
{
    private static int CRANE_MAXIMUM_HEIGHT = 4400;
    private static double CRANE_MOVEMENT_SPEED_UP = 0.7;
    private static double CRANE_MOVEMENT_SPEED_DOWN = 0.7;
    private static double CRANE_HOLD_POWER_INCREASE = 0.0005;
    private static double CRANE_HOLD_POWER_DECREASE = 0.00025;
    private static double CRANE_TICKS_PER_INCH = 185;
    private static double WHISKER_SERVO_INIT_POSITION = 0;

    private static double SPIKE_SERVO_GRAB = 0.75;
    private static double SPIKE_SERVO_RELEASE = 0.90;
    private static double LEFT_PAW_OPEN = 1;
    private static double RIGHT_PAW_OPEN = 0;
    private static double LEFT_PAW_CLOSE = 0.5;
    private static double RIGHT_PAW_CLOSE = 0.5;
    private static double SPIKE_LIFT_DOWN = 0.36;
    private static double SPIKE_LIFT_UP = 0;
  
    private long loopCount=0;
    private Servo spikeServo;
    private Servo leftPaw, rightPaw;
    private Servo spikeLift;
    private Servo whiskerServo;
    
    private boolean shouldCraneBeStopped=true;
    private DcMotor craneMotor;
    private int previousCranePosition;
    private TouchSensor craneTouchSensor;
    int craneMotor_minPositionSeen;
    boolean craneOverride=false;
    
    public TouchSensor whisker;
    
    public TouchSensor leftPawLimitSwitch, rightPawLimitSwitch;
    
    private DcMotor m0;
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor driveMotors[];
    
    double xPos=0, yPos=0;
    
    double forwardPowers[] = new double[] {-1.0, -1.0, +1.0, +1.0};
    double backwardPowers[] = new double[] {+1.0, +1.0, -1.0, -1.0};
    double leftPowers[] = new double[] {-1.0, +1.0, -1.0, +1.0};
    double rightPowers[] = new double[] {+1.0, -1.0, +1.0, -1.0};
    
    // Gyro variables
    BNO055IMU imu;
    
    // Forward is zero, 360, etc
    double totalDegreesTurned = 90;
    Double desiredTotalDegreesTurned = null;

    // Remember the previous direction so we know how much we turned
    double previousDirection = 0;


    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    final float[] hsvValues = new float[3];
    
    LinearOpMode opMode;
    
    String robotStatus = null;
    List<String> robotMessages=new ArrayList<String>();


    public TeamRobot(LinearOpMode opMode)
    {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        m0 = hardwareMap.dcMotor.get("m0");
        m1 = hardwareMap.dcMotor.get("m1");
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
       
        m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        driveMotors = new DcMotor[] {m0, m1, m2, m3};

        craneMotor = hardwareMap.dcMotor.get("crane");
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor_minPositionSeen = craneMotor.getCurrentPosition();
        craneMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        previousCranePosition = craneMotor.getCurrentPosition();
        
        spikeServo = hardwareMap.servo.get("spike");
        spikeServo.setPosition(SPIKE_SERVO_RELEASE);
        craneTouchSensor = hardwareMap.get(TouchSensor.class, "crane_touch");
        
        whisker = hardwareMap.get(TouchSensor.class, "whiskerTouch");
        whiskerServo = hardwareMap.servo.get("whiskerServo");
        whiskerServo.setPosition(WHISKER_SERVO_INIT_POSITION);
        
        
        leftPaw = hardwareMap.servo.get("left_paw");
        rightPaw = hardwareMap.servo.get("right_paw");
        leftPawLimitSwitch = hardwareMap.get(TouchSensor.class, "left_paw_limit");
        rightPawLimitSwitch = hardwareMap.get(TouchSensor.class, "right_paw_limit");
        openPaws();
        
        spikeLift = hardwareMap.servo.get("spike_lift");
        spikeUp();

        
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        //colorSensor.enableLed(false);

        opMode.telemetry.addData("Controls: ", "%s", this::getControlTelemetry);

        opMode.telemetry.addData("Drive: ", "%s", this::getDriveTelemetry);
        opMode.telemetry.addData("Color:", "%s", this::getColorTelemetry);
        
        opMode.telemetry.addData("Crane:", "%s", this::getCraneTelemetry);
        opMode.telemetry.addData("Paws:", "%s", this::getPawTelemetry);
        opMode.telemetry.addData("SpikeLift:", "%s", this::getSpikeLiftTelemetry);
        

        // Initialize IMU/Gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        _getAngle1FromImu();
        opMode.telemetry.addData("IMU: ", "%s", this::getImuTelemetry);
        
        opMode.telemetry.addData("Loop status", "%s", this::getRobotStatusMessage);
        opMode.telemetry.addData("Messages", "%s", this::getRobotMessages);

    }



    public String getControlTelemetry(){
        return String.format("GP1.L_J=[%.1f,%.1f], GP1.R_J=[%.1f, %.1f]\nGP2.L_T: %.2f", 
            opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y,
            opMode.gamepad1.right_stick_x, opMode.gamepad1.right_stick_y,
            opMode.gamepad2.left_trigger);
    }
    
    public String getSpikeLiftTelemetry(){
        return String.format("SpikeLift: %.2f", spikeLift.getPosition());
    }
    
    public void openPaws(){
        leftPaw.setPosition(LEFT_PAW_OPEN);
        rightPaw.setPosition(RIGHT_PAW_OPEN);
    }
    
    public void closePaws(){
        leftPaw.setPosition(LEFT_PAW_CLOSE);
        rightPaw.setPosition(RIGHT_PAW_CLOSE);
    }
    
    public void setPawLocation(double left, double right){
        leftPaw.setPosition(left);
        rightPaw.setPosition(right);
    }
    
    public void setWhiskerLocation(double position){
        whiskerServo.setPosition(position);
    }
    
    public double getWhiskerLocation(){
        return whiskerServo.getPosition();
    }
    
    public void setSpikeLiftPosition(double position){
        spikeLift.setPosition(position);
    }
    
    public double getSpikeLiftPosition(){
        return spikeLift.getPosition();
    }
    
    public void spikeUp(){
        setSpikeLiftPosition(SPIKE_LIFT_UP);
    }
    
    public void spikeDown(){
        setSpikeLiftPosition(SPIKE_LIFT_DOWN);
    }
    
    public void spike_switchPosition(){
        if (spikeLift.getPosition() == SPIKE_LIFT_UP)
            setSpikeLiftPosition(SPIKE_LIFT_DOWN);
        else
            setSpikeLiftPosition(SPIKE_LIFT_UP);
    }
    
    public String getRobotStatusMessage() {
        if (robotStatus==null )
            return "none";
        else
            return robotStatus;
    }
    
    public String getRobotMessages() {
        if (robotMessages.size()==0)
            return "none";
        else
            return robotMessages.toString();
    }

    public void setRobotStatusMessage(String messageFormat, Object... args) {
        robotStatus = String.format(messageFormat, args);
    }
    
    public void addRobotMessage(String messageFormat, Object... args) {
        robotMessages.add(String.format(messageFormat, args));
    }

    public void loopWhileWaiting(String messageFormat, Object... args){
        setRobotStatusMessage(messageFormat, args);
        loop();
    }
    
    public void loop()
    {
        loopCount++;
        opMode.telemetry.update();
        robotMessages.clear();
        
        addRobotMessage("Loop #%07d", loopCount);
        if (craneTouchSensor.isPressed() 
            || craneMotor.getCurrentPosition() < craneMotor_minPositionSeen)
        {
            craneSetCurrentPositionAsMinimum();
        }
            
        if ( ! craneOverride ) {
            if (craneMotor.getPower() < 0 && !canCraneGoDown()) {
                addRobotMessage("Safety: Stopping crane at bottom");
                craneStop();
            }
            
            if (craneMotor.getPower() > 0 && !canCraneGoUp()) {
                addRobotMessage("Safety: Stopping crane at top");
                craneStop();
            }
        }       
        
        int currentCranePosition = craneMotor.getCurrentPosition();

        int craneChange = currentCranePosition - previousCranePosition;
        previousCranePosition = currentCranePosition;
        if (shouldCraneBeStopped) {
            if ( craneChange<0 )
                craneMotor.setPower(CRANE_HOLD_POWER_INCREASE + craneMotor.getPower());
            else if ( craneChange>0)
                craneMotor.setPower(CRANE_HOLD_POWER_DECREASE + craneMotor.getPower());
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
    }
    
    public boolean shouldRobotBeRunning() {
        if (opMode.isStopRequested())
            return false;
        else
            return true;
    }
    public String getColorTelemetry() {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return String.format("\nRed: %4.3f Gre: %4.3f Blu: %4.3f \nHue: %4.1f Sat: %4.3f Val: %4.3f \nAlf: %4.3f\nZone: %d", 
            colors.red, colors.green, colors.blue, hsvValues[0], hsvValues[1], hsvValues[2], colors.alpha, findParkingZone());
    }
    
    public int findParkingZone() {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        double red = colors.red, blue = colors.blue, green = colors.green;
        if (red + blue + green > 1) 
            return 3;
        if (red > blue) 
            return 2;
        return 1;
    }
    
    public void craneSetCurrentPositionAsMinimum() {
        // Check if this was already the minimum position
        if ( craneMotor_minPositionSeen == craneMotor.getCurrentPosition() )
            return;
            
        addRobotMessage("Update: Updating crane's minimum position");
        craneMotor_minPositionSeen = craneMotor.getCurrentPosition();
    }
    
    public int getCraneHeight() {
        return craneMotor.getCurrentPosition() - craneMotor_minPositionSeen;
    }
    
    public boolean canCraneGoDown() {
        if (craneTouchSensor.isPressed())
            return false;
        return getCraneHeight() > craneMotor_minPositionSeen;
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
            setCraneMotor_raw(CRANE_MOVEMENT_SPEED_UP/2);
            return;
        }
        
        if (!canCraneGoUp()) {
            craneStop();
            return;
        }
            
        setCraneMotor_raw(CRANE_MOVEMENT_SPEED_UP);
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
            setCraneMotor_raw(-CRANE_MOVEMENT_SPEED_DOWN/2);
            return;
        }

        if (!canCraneGoDown()) {
            craneStop();
            return;
        }
            
        setCraneMotor_raw(-CRANE_MOVEMENT_SPEED_DOWN);
        
    }
    
    private void setCraneMotor_raw(double cranePower) {
        craneMotor.setPower(cranePower);
        shouldCraneBeStopped=false;
    }
    
    public void craneStop()
    {
        // Is this the first time we're stopping robot?
        if ( !shouldCraneBeStopped ) {
            shouldCraneBeStopped=true;
            craneMotor.setPower(0);
        }
    }
    
    public void spikeGrab()
    {
        spikeServo.setPosition(SPIKE_SERVO_GRAB);
        
    }
    
    public void spikeRelease()
    {
        spikeServo.setPosition(SPIKE_SERVO_RELEASE);
        
    }
    

    public void movecrantoposition(double height_inches)
    {
        int height_ticks = (int) (185.5 * height_inches) ; 
        if(height_ticks == 0){
            craneDown();
            while(shouldRobotBeRunning() && craneTouchSensor.isPressed()==false)
                loopWhileWaiting("Moving elevator to %.2f inches, waiting for press", 
                    height_inches);
                
            craneStop();
        }
        else if(getCraneHeight() >= height_ticks){
            craneDown();
            while(shouldRobotBeRunning() && getCraneHeight() > height_ticks)
                loopWhileWaiting("Moving elevator to %.2f inches, %d clicks left", 
                    height_inches, getCraneHeight()-height_ticks);
            craneStop();
        } else {
            craneUp();
            while(shouldRobotBeRunning() && getCraneHeight() <= height_ticks)
                loopWhileWaiting("Moving elevator to %.2f inches, %d clicks left", 
                    height_inches, height_ticks-getCraneHeight());
            craneStop();
        }
        robotStatus = null;
    }
    
    //USE : returns list of unscaled moter powers (must in put quadrant)
    //GOAL : Shouldn't consider the quadrant
    public double[] getPowerRatiosInQuadrant(int quadrantNumber, double angleIntoQuadrant, double zeroDegPowers[], double ninetyDegPowers[])
    {
        if (angleIntoQuadrant<0) {
            setRobotStatusMessage("AngleIntoQuadrant must be >=0, not %.1f", angleIntoQuadrant);
            return null;
        }
        if (angleIntoQuadrant>90) {
            setRobotStatusMessage("AngleIntoQuadrant must be <=90, not %.1f", angleIntoQuadrant);
            return null;
        }

        double zeroFraction = (90.0 - angleIntoQuadrant) / 90.0;
        double ninetyFraction = angleIntoQuadrant / 90.0;
       
        double result[] = new double[4];
        for (int motorNumber = 0; motorNumber < 4; motorNumber++)
        {
            result[motorNumber] = zeroDegPowers[motorNumber] * zeroFraction + ninetyDegPowers[motorNumber] * ninetyFraction;
            //result[motorNumber] = 1.0 * (45/90) + -1.0 * (45/90)
        }
        
        opMode.telemetry.addData("MotorPowersForDirection", 
            "Quadrant %d|Angle into quadrant=%.1f|Angle0 Powers=[%+.0f,%+.0f,%+.0f,%+.0f]|Angle90 Powers=[%+.0f,%+.0f,%+.0f,%+.0f]|ResultingPowers=[%+.0f,%+.0f,%+.0f,%+.0f]",
                quadrantNumber, angleIntoQuadrant, 
                zeroDegPowers[0], zeroDegPowers[1], zeroDegPowers[2], zeroDegPowers[3],
                ninetyDegPowers[0], ninetyDegPowers[1], ninetyDegPowers[2], ninetyDegPowers[3],
                result[0], result[1], result[2], result[3]);

        return result;
    }
    
    //USE : returns a list of unscaled moter powers (works for all quadrants)
    //GOAL : Sould scaled moter powers and should consider quadrant
    public double[] getMotorPowersForDirection(double joystickAngle)
    {
        //test
        double driveAngle = joystickAngle - getHeading() + 90;
        // driveAngle will be between -269.9 --> +449.9
        // Fix driveAngle so it is in the range 0 --> 359.9
        if (driveAngle >= 360)  
            driveAngle = (driveAngle - 360);
        if (driveAngle < 0)
            driveAngle = (driveAngle + 360);
        opMode.telemetry.addData("joystickAngle", "%s", joystickAngle);
        opMode.telemetry.addData("driveAngle", "%s", driveAngle);
        
        if (driveAngle <= 90)
        {
            return getPowerRatiosInQuadrant(1, driveAngle, rightPowers, forwardPowers);
        }
        else if (driveAngle <= 180)
        {
            return getPowerRatiosInQuadrant(2, driveAngle - 90, forwardPowers, leftPowers);
        }
        else if (driveAngle <= 270)
        {
            return getPowerRatiosInQuadrant(3, driveAngle - 180, leftPowers, backwardPowers);
        }
        else
        {
            return getPowerRatiosInQuadrant(4, driveAngle - 270, backwardPowers, rightPowers);
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
        
        // Clear desiredTotalDegreesTurned if a spin was specified   
        for (int i=0; i< 4 ;i++) {
            if (spinPowers[i] != 0)
                desiredTotalDegreesTurned=null;
        }
        if (desiredTotalDegreesTurned != null) {
            double headingError = (desiredTotalDegreesTurned - totalDegreesTurned);
            
            double correction;
            if (Math.abs(headingError) < 0.5)
                correction = 0;
            else if (Math.abs(headingError) > 30)
                correction = 1;
            else 
                correction = Math.abs(headingError/30);
                
            if (headingError>0)
                spinPowers = new double [] {correction, correction, correction, correction};
            else if (headingError<0)
                spinPowers = new double [] {-correction, -correction, -correction, -correction};
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
            double desiredPower = tempMotorPowers[i];
            // Motor 2 is too fast when measured by a tachometer
            if (i==2)
              desiredPower = desiredPower * 76.0/81.0;
            driveMotors[i].setPower(desiredPower);
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
        return String.format("Pos:[min = %4d, current = %4d, allowed max = %4d] h=%4d chg=%+d stopped=%S touch:%s|whisker: %s|Pow: %4.2f override:%s|Barb:%s (%.2f)",
            craneMotor_minPositionSeen,
            craneMotor.getCurrentPosition(),
            craneMotor_minPositionSeen+CRANE_MAXIMUM_HEIGHT,
            getCraneHeight(),
            craneMotor.getCurrentPosition()-previousCranePosition,
            shouldCraneBeStopped ? "Yes": "No",
            craneTouchSensor.isPressed() ? "Pressed  " : "Unpressed",
            whisker.isPressed() ? "Yes  " : "No",
            craneMotor.getPower(),
            craneOverride ? "Y" : "N",
            spikeServo.getPosition()==SPIKE_SERVO_GRAB ? "Grab" : "Release",
            spikeServo.getPosition());
    }
    
    public String getPawTelemetry() {
        return String.format("L: %.2f %7s | R: %.2f %7s", 
        leftPaw.getPosition(), leftPawLimitSwitch.isPressed() ? "TOUCH" : "notouch",
        rightPaw.getPosition(),rightPawLimitSwitch.isPressed() ? "TOUCH" : "notouch");
    }
    
    public void moveForTime(double angle, long msec, double power){
        long timeStarted = System.currentTimeMillis();
            if (desiredTotalDegreesTurned == null)
                desiredTotalDegreesTurned = totalDegreesTurned;
            long endTime = timeStarted + msec;
            
            while (shouldRobotBeRunning() && System.currentTimeMillis()<endTime) {
                double motorPowers[] = getMotorPowersForDirection(angle);
                setMotorPowers(power, motorPowers, null);
                loopWhileWaiting("Moving for %.2f seconds in direction %.0f, pow=%.2f. %.2f seconds left", 
                    1.0 * msec/1000, angle, power,
                    1.0*endTime-System.currentTimeMillis());
                
            }
            robotStatus=null;
            
    }
    
    public void moveForTime(double angle, long msec){
        moveForTime(angle, msec, 0.5);
    }
    
    public void spin(double spinPower) {
        setMotorPowers(spinPower, null, new double[] {spinPower,spinPower,spinPower,spinPower});
    }
    
    public void turnForAngle(double turnAngle, double turnSpeed){
        double startAngle = totalDegreesTurned;
        if (desiredTotalDegreesTurned == null)
            desiredTotalDegreesTurned = totalDegreesTurned;
        desiredTotalDegreesTurned += turnAngle;
        
        if (turnAngle < 0) {
            turnSpeed = turnSpeed * -1;
        }
        spin(turnSpeed);
        while (shouldRobotBeRunning() && Math.abs(totalDegreesTurned - startAngle) < Math.abs(turnAngle))
            loop();
            
    }
    
    public void stop() {
        setMotorPowers(0, null, null);
    }
    
    public void waitForTime(long msec){
        long timeStarted = System.currentTimeMillis();
            
            long endTime = timeStarted + msec;
            
            while (shouldRobotBeRunning() && System.currentTimeMillis()<endTime) {
                loop();
                loopWhileWaiting("Waiting for %.2f seconds. %.2f seconds left", 
                    1.0 * msec/1000, 
                    1.0*endTime-System.currentTimeMillis());
            }
           robotStatus = null; 
    }
    
    public void makeSquare(long pauseSecs, double turnAngle, double startAngle){
        for(int i = 0; i < 4; i++){
            moveForDistance(startAngle, 10, 0);
            waitForTime(pauseSecs);
            turnForAngle(turnAngle, 0.5);
            startAngle += 90;
        }
        
    }
    
    private double TICKS_PER_INCH = 84;
    
    private void moveForDistance_90(double angle, double distanceInInches, double headingChange){
        int startPosition0 = m0.getCurrentPosition();
        int startPosition1 = m1.getCurrentPosition();
        int startPosition2 = m2.getCurrentPosition();
        int startPosition3 = m3.getCurrentPosition();
        
        if (desiredTotalDegreesTurned == null)
            desiredTotalDegreesTurned = totalDegreesTurned;
        double desiredTotalDegreesTurned_start = desiredTotalDegreesTurned;
        double motorPowers[] = getMotorPowersForDirection(angle);

        int distanceInTicks = (int) (distanceInInches * TICKS_PER_INCH);
        double distanceTravelled = 0;
        
        while (shouldRobotBeRunning() && distanceTravelled < distanceInTicks) {
                setMotorPowers(0.5, motorPowers, null);
                distanceTravelled = Math.abs(m0.getCurrentPosition() - startPosition0);
                distanceTravelled += Math.abs(m1.getCurrentPosition() - startPosition1);
                distanceTravelled += Math.abs(m2.getCurrentPosition() - startPosition2);
                distanceTravelled += Math.abs(m3.getCurrentPosition() - startPosition3);
                distanceTravelled /= 4;
                
                desiredTotalDegreesTurned = desiredTotalDegreesTurned_start + headingChange * distanceTravelled / distanceInTicks;
                
                loopWhileWaiting("moving for distance. %.0f clicks left", distanceInTicks - distanceTravelled);
        }
        desiredTotalDegreesTurned = desiredTotalDegreesTurned_start + headingChange;    
    }
    
    private void moveForDistance_45(double angle, double distanceInInches, double headingChange){
        double startHeading = getHeading();
        
        int startPosition0 = m0.getCurrentPosition();
        int startPosition1 = m1.getCurrentPosition();
        int startPosition2 = m2.getCurrentPosition();
        int startPosition3 = m3.getCurrentPosition();
        
        if (desiredTotalDegreesTurned == null)
            desiredTotalDegreesTurned = totalDegreesTurned;
        double desiredTotalDegreesTurned_start = desiredTotalDegreesTurned;
    
        double motorPowers[] = getMotorPowersForDirection(angle);
        setMotorPowers(0.5, motorPowers, null);

        int distanceInTicks = (int) (distanceInInches * TICKS_PER_INCH);
        double distanceTravelled = 0;
        while (shouldRobotBeRunning() && distanceTravelled < distanceInTicks) {
                distanceTravelled = Math.abs(m0.getCurrentPosition() - startPosition0);
                distanceTravelled += Math.abs(m1.getCurrentPosition() - startPosition1);
                distanceTravelled += Math.abs(m2.getCurrentPosition() - startPosition2);
                distanceTravelled += Math.abs(m3.getCurrentPosition() - startPosition3);
                distanceTravelled /= 2;
                // distance is divided by two because two motors are stationary when moving at 45 degrees
                
                desiredTotalDegreesTurned = desiredTotalDegreesTurned_start + headingChange * distanceTravelled / distanceInTicks;
                
                loop();
        }
        
        desiredTotalDegreesTurned = desiredTotalDegreesTurned_start + headingChange;  
    }
    
    public void moveForDistance(double angle, double distanceInInches, double headingChange){
        if (angle % 90 == 0)
            moveForDistance_90(angle, distanceInInches, headingChange);
        else if (angle % 45 == 0){
            moveForDistance_45(angle, distanceInInches, headingChange);
        }
        else {
            throw new IllegalArgumentException("can only move in the 45 or 90 degree directions");
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
    
    // [0,360) where 0 is 'North', 90 is 'West', 180 is 'South' and 270 is 'East'
    // North is defined to be the direction the robot was facing when it started
    // or reset to
    public double getHeading() {
        double heading = totalDegreesTurned % 360;
        if ( heading>=0 )
            return heading;
        else
            return heading+360;
    }
    
    public void resetHeading_North() {
        totalDegreesTurned = 90;
    }
    
    public void resetHeading_South() {
        totalDegreesTurned = 270;
    }
    
    public void resetHeading_East() {
        totalDegreesTurned = 0;
    }
    
    public void resetHeading_West() {
        totalDegreesTurned = 180;
    }
    
    public String getImuTelemetry() {
        return String.format("TotalDegreesTurned=%+.0f | Heading=%.0f | DesiredDegreesTurned: %.0f | A1=%.1f | A2=%.1f | A3=%.1f",
            totalDegreesTurned,
            getHeading(),
            desiredTotalDegreesTurned,
            _getAngle1FromImu(),
            _getAngle2FromImu(),
            _getAngle3FromImu());
    }

    public double getRobotAngle() {
        return _getAngle1FromImu();
        
    }
}
