package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Auto_RobotTest extends LinearOpMode {

    TeamRobot robot;
    public void runOpMode() 
    {
        robot = new TeamRobot(this);

        while (!isStarted())
        {
           robot.loop();
        }
        
        while (!isStopRequested())
        {
            robot.makeSquare(2000, 90, 90);
            robot.waitForTime(2000);
            robot.makeSquare(2000, 45, 90);
            robot.stop();
        }
    }
}
