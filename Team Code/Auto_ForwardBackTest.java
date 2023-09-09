package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class Auto_ForwardBackTest extends LinearOpMode {

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
           robot.moveForTime (90,10000);
           robot.stop();
           robot.waitForTime(500);
           // This was a syntax error, and needs to be fixed
            //robot.turnForAngle(180);
            robot.stop();
            robot.waitForTime(500);
            
        }
       
    }
}
