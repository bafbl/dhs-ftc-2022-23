package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Auto_Blue_Right", group="Training")
//@Disabled
public class Auto_Blue_Right extends LinearOpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2022 robot   = new MaristBaseRobot2022();   
    private ElapsedTime runtime = new ElapsedTime();
    
    // Variables to control Speed
    double velocity = 0.5; // Default velocity


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Perform the steps of Autonomous One Step at a Time.
        // MARIST: Add Code here!
        // Available Calls: forward(inches),delay(seconds),right(degrees),left(degrees)
        // robot.leftHand.setPosition(), robot.rightHand.setPosition()
        // Engineering Java Calls:
        // robot.moveDistance(inches, speed)
        // robot.turnAngle(degrees, speed)
        
        robot.moveDistance(12, 0.5);
        delay(1);
        robot.moveDistance(-12, 0.5);
        delay(1);
        

        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);
    }

    // Functions for REACH 2019 based on Python Turtles
    public void forward(double inches)
    {
        robot.driveStraightInches(velocity, inches, 10);
    }
    
    public void right(double degrees)
    {
        robot.pointTurnDegrees(velocity, degrees, 10);
    }
    
    public void left(double degrees)
    {
        degrees = degrees * -1;
        robot.pointTurnDegrees(velocity, degrees, 10);
    }
    
    public void speed(int speed)
    {
        double newSpeed = (double)speed / 10.0;
        velocity = newSpeed;
    }
    
    // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    // REACH: Add Functions Here
    

}
