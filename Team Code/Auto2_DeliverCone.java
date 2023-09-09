package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Auto2_DeliverCone extends LinearOpMode {

    TeamRobot robot;
    // todo: write your code here
    public void runOpMode()
    {
        robot = new TeamRobot(this);

        while (!isStarted())
        {
           robot.loop();
        }
        robot.setCraneOverride(true);
        robot.movecrantoposition(0);
        while (!isStopRequested())
        {
            robot.movecrantoposition(10);
            robot.movecrantoposition(0);
        }
       
    }
}
