package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Autonomous_Spin extends LinearOpMode {

    TeamRobot robot;

    // todo: write your code here
    public void runOpMode()
    {
        robot = new TeamRobot(this);
        
        while (!isStarted())
        {
            robot.loop();
            
        }
        
        robot.setMotorPowers(1.0, null, new double[] {-1, -1, -1, -1} );
        
            long timeStarted = System.currentTimeMillis();
            
                long endTime = timeStarted + 15000;
                
                while (System.currentTimeMillis()<endTime) {
                    robot.loop();
                }
        
     }
        
}
