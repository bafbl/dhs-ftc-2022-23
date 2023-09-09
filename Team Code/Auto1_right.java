package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Auto1_right extends LinearOpMode {

    TeamRobot robot;
    // todo: write your code here
    public void runOpMode()
    {
        robot = new TeamRobot(this);

        while (!isStarted())
        {
           robot.loop();
        }
        
        while (!isStopRequested())
        {
            robot.craneServoGrab();
            robot.movecrantoposition(6);
            robot.moveForTime(0, 2*1000);
    
            robot.setMotorPowers(0, null, null);
            robot.movecrantoposition(1);
            break;
            
        }
       
    }
}
