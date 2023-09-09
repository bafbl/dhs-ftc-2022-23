package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Auto2_DeliverConeRight extends LinearOpMode {

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
            robot.movecrantoposition(15);
            robot.moveForTime(90, 750);
            robot.setMotorPowers(0, null, null);
            robot.waitForTime(1000);
            robot.craneServoRelease();
            robot.waitForTime(1000);
            robot.moveForTime(270, 1000);
            robot.moveForTime(0, 1200);
            robot.moveForTime(90, 2500);
            break;
        }
       
    }
}
