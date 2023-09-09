package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class PawTestTeleOp extends LinearOpMode
{
   private TeamRobot robot;
   public void runOpMode()
    {
        robot = new TeamRobot(this);
        
        while (!isStarted())
        {
          robot.loop();
        }
        
        while (!isStopRequested())
        {
          robot.loop();
          robot.setPawLocation(gamepad1.left_stick_x, gamepad1.right_stick_x);
          if (gamepad1.dpad_left)
            robot.setWhiskerLocation(Range.clip(robot.getWhiskerLocation() - 0.05, 0, 1));
          else if (gamepad1.dpad_right)
            robot.setWhiskerLocation(Range.clip(robot.getWhiskerLocation() + 0.05, 0, 1));
            
          if (gamepad1.y)
            robot.setSpikeLiftPosition(Range.clip(robot.getSpikeLiftPosition() + 0.05, 0, 1));
          else if (gamepad1.a)
            robot.setSpikeLiftPosition(Range.clip(robot.getSpikeLiftPosition() - 0.05, 0, 1));
        }
    }
    
}
