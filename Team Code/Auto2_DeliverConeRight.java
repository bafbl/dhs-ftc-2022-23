package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Auto2_DeliverConeRight extends LinearOpMode {

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
            robot.craneServoGrab();
            robot.movecrantoposition(15);
            robot.moveForTime(180, 8500/12);
            robot.moveForTime(90, 750);
            robot.setMotorPowers(0, null, null);
            robot.movecrantoposition(13);
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
