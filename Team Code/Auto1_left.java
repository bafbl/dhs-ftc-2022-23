package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Auto1_left extends LinearOpMode {

    TeamRobot robot;
    // todo: write your code here
    // first league match
    public void runOpMode()
    {
        robot = new TeamRobot(this);

        while (!isStarted())
        {
           robot.loop();
        }
        
        while (!isStopRequested())
        {
            // picks up crane
            robot.spikeGrab();
            robot.movecrantoposition(6);
            //moves to left terminal
            robot.moveForTime(180, 2*1000);
            
            //stops the robot
            robot.setMotorPowers(0, null, null);
            robot.movecrantoposition(1);
            break;
            
        }
       
    }
}
