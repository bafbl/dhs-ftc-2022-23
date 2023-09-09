package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;


public class RobotLocationTracker {
    double robotX, robotY;
    
    Long prevWheelLoc0, prevWheelLoc1, prevWheelLoc2, prevWheelLoc3;
    public enum MovementDirection {N, NE, E, SE, S, SW, W, NW};
    public enum PivotDirection {CW, CCW};
    
    public enum WheelConfig {
        W0(MovementDirection.SE, MovementDirection.NW,
           PivotDirection.CCW, PivotDirection.CW),
        W1(MovementDirection.SW, MovementDirection.NE,
           PivotDirection.CCW, PivotDirection.CW),
        W2(MovementDirection.NE, MovementDirection.SW,
           PivotDirection.CW, PivotDirection.CCW),
        W3(MovementDirection.NW, MovementDirection.SE,
           PivotDirection.CW, PivotDirection.CCW);
           
        MovementDirection positiveMovementDirection, negativeMovementDirection;
        PivotDirection positivePivotDirection, negativePivotDirection;
        
        WheelConfig(MovementDirection positiveMovementDirection, 
                MovementDirection negativeMovementDirection,
                PivotDirection positivePivotDirection, 
                PivotDirection negativePivotDirection)
        {
            this.positiveMovementDirection = positiveMovementDirection;
            this.negativeMovementDirection = negativeMovementDirection;
            this.positivePivotDirection = positivePivotDirection;
            this.negativePivotDirection = negativePivotDirection;
        }
        
        MovementDirection getMovementDirectionForWheelChange(int wheelChange) {
            if (wheelChange>0)
                return positiveMovementDirection;
            else
                return negativeMovementDirection;
        }
        PivotDirection getPivotDirectionForWheelChange(int wheelChange) {
            if (wheelChange>0)
                return positivePivotDirection;
            else
                return negativePivotDirection;
        }
    }

    public void updateRobotLocation(long wheelLoc0, long wheelLoc1, long wheelLoc2, long wheelLoc3) {
        if (prevWheelLoc0 == null) {
            prevWheelLoc0=wheelLoc0;
            prevWheelLoc1=wheelLoc1;
            prevWheelLoc2=wheelLoc2;
            prevWheelLoc3=wheelLoc3;
            return;
        }
        
        long wheelDelta0 = wheelLoc0 - prevWheelLoc0, 
            wheelDelta1 = wheelLoc1 - prevWheelLoc1,
            wheelDelta2 = wheelLoc2 - prevWheelLoc2,
            wheelDelta3 = wheelLoc3 - prevWheelLoc3;
            
        //compesate for opposite mounting
        wheelDelta2 *= -1;
        
    }
    
}
