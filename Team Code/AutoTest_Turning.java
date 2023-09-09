package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutoTest_Turning extends LinearOpMode {

    TeamRobot robot;
    // todo: write your code here
    public void runOpMode()
    {
        robot = new TeamRobot(this);

        while (!isStarted())
        {
           robot.loop();
        }
        robot.turnForAngle(90, 0.25);
        while (!isStopRequested()) {
            robot.stop();
            robot.loop();
        }
    }
}
