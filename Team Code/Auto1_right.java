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
            robot.spikeGrab();
            robot.movecrantoposition(6);
            
            
            break;
            
        }
       
    }
}
