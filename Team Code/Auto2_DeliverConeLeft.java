package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Auto2_DeliverConeLeft extends LinearOpMode {

    TeamRobot robot;
    // todo: write your code here
    // 11/19 league match
    public void runOpMode()
    {
        robot = new TeamRobot(this);

        while (!isStarted())
        {
           robot.loop();
        }
        
        while (!isStopRequested())
        {
            // grab and lift cone
            robot.craneServoGrab();
            robot.movecrantoposition(15);
            // move into dropping position
            robot.moveForTime(0, 8500/12);
            robot.moveForTime(90, 750);
            // stop and wait to release cone
            robot.setMotorPowers(0, null, null);
            robot.movecrantoposition(13);
            robot.waitForTime(1000);
            robot.craneServoRelease();
            robot.waitForTime(1000);
            // move back, to the side, and forward to park
            robot.moveForTime(270, 500);
            robot.moveForTime(180, 800);
            robot.moveForTime(90, 2500);
            
            
            break;
        }
       
    }
}
