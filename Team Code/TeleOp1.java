package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp

public class TeleOp1 extends LinearOpMode
{
    private TeamRobot robot;
    //USE : returns moter power scale
    double getPowerScale(double joystickX, double joystickY)
    {
        /*
        return Math.pow(1.3, Math.sqrt(Math.pow(joystickY, 2) + Math.pow(joystickX, 2))) - 1;
        */
        
        double hyp = Math.sqrt(Math.pow(joystickY, 2) + Math.pow(joystickX, 2));
        
        if(hyp < 0.3)
        {
            hyp /= 2;
        }
        
        return hyp;
    }
    
    //USE : returns joystick angle in degrees
    double getAngleInDegrees(double joystickX, double joystickY)
    {
        double angleInRadians = Math.atan2(joystickY, joystickX);
        double angleInDegrees = angleInRadians * 180 / Math.PI;
            
        if (angleInDegrees < 0)
        {
                angleInDegrees += 360;
        }
        return angleInDegrees;
    }

    public void driveRobotFromGamepad(Gamepad gamepad, double powerScale) {
        double joystickY = -gamepad.left_stick_y;
        double joystickX = gamepad.left_stick_x;
            
        boolean fastMode = gamepad.right_bumper;
            
        double directionPowers[] = robot.getMotorPowersForDirection(getAngleInDegrees(joystickX, joystickY));
        double spinControl = gamepad.right_stick_x;
            
        double spinPowers[] = new double[] {-spinControl, -spinControl, -spinControl, -spinControl};
            
        double joystickSpeed = getPowerScale(joystickX, joystickY);
            
        double speed;
            
        if (fastMode)
          speed = powerScale * joystickSpeed;
        else 
          speed = powerScale * 0.5 * joystickSpeed;
            
        //Run
        robot.setMotorPowers(speed, directionPowers, spinPowers );  
    }
    
    //USE : init
    public void runOpMode()
    {
        robot = new TeamRobot(this);
        
        while (!isStarted())
        {
           robot.loop();
        }
        
        while (!isStopRequested())
        {
            robot.loop();


            // If gamepad2 is being used, drive the robot from it
            if ( Math.abs(gamepad2.left_stick_y) > 0.1 
              || Math.abs(gamepad2.left_stick_x) > 0.1
              || Math.abs(gamepad2.right_stick_x)> 0.1
              || Math.abs(gamepad2.right_stick_y)> 0.1 )
            {
                driveRobotFromGamepad(gamepad2, 0.5);
            }
            else // Drive the robot from gamepad1
            {
                driveRobotFromGamepad(gamepad1, 1.0);
            }
            
            if(gamepad2.y)
              robot.craneUp(gamepad2.right_bumper);
            else if(gamepad2.a)
              robot.craneDown(gamepad2.right_bumper);
            else
              robot.craneStop();
              
            if ( gamepad2.left_bumper )
              robot.craneSetCurrentPositionAsMinimum();
              
            if(gamepad2.b)
                robot.craneServoGrab();
            if(gamepad2.x)
                robot.craneServoRelease();
                
        }
       
    }
}
   
   
