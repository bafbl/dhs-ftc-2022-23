package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutoDistMeasurements extends LinearOpMode {

    TeamRobot robot;
    public void runOpMode() 
    {
        robot = new TeamRobot(this);

        while (!isStarted())
        {
           robot.loop();
        }
        
        robot.moveForDistance(90, 10, 0);
        robot.stop();
        robot.waitForTime(100000);
        
    }
}
