package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Auto3_DeliverConeRight extends LinearOpMode {

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
            // grab and lift cone
            robot.movecrantoposition(1);
            robot.craneServoGrab();
            robot.waitForTime(250);
            robot.movecrantoposition(15);
            // move into dropping position
            robot.moveForTime(180, 8500/12);
            robot.moveForTime(90, 750);
            // stop and wait to release cone
            robot.setMotorPowers(0, null, null);
            robot.movecrantoposition(13);
            robot.waitForTime(1000);
            robot.craneServoRelease();
            robot.waitForTime(500);
            robot.craneServoGrab();
            robot.waitForTime(100);
            robot.craneServoRelease();
            robot.waitForTime(100);
            robot.craneServoGrab();
            robot.waitForTime(1000);
            robot.craneServoRelease();
            robot.movecrantoposition(16);
            robot.moveForTime(270, 1000);
            robot.stop();
            robot.movecrantoposition(0);
            // move back, to the side
            robot.moveForTime(0, 1250);
            robot.turnForAngle(180, 0.3);
            //robot.moveForTime(90, 1000);

            
            // move forward slowly up to the signal cone
            robot.moveForTime(270, 4250, 0.25);
            robot.stop();
            robot.waitForTime(1000);
            // read signal cone and park in zone
            int zone = robot.findParkingZone();
            robot.moveForTime(270, 1500, 0.25);
            if (zone == 1) {
                robot.moveForTime(0, 2000);
            }
            else if (zone == 3) {
                robot.moveForTime(180, 2000);
            }
            break;
        }
       
    }
}
