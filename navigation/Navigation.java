package org.firstinspires.ftc.teamcode.navigation; 

public class Navigation {
    //Navigation class tells the Robot to move to a specific Pose, using a specific Path

    private Pose startPose = null;
    private DriveTrain driveTrain = null;
    
    public void init(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }

    public void goToPose(Pose endPose, Path chosenPath, double power) {
        if(chosenPath == Path.XY){
                driveTrain.strafe(endPose.x, power);
                driveTrain.move(endPose.y, power);
                driveTrain.turn(endPose.orientation, power);
        }
        else if(chosenPath == Path.YX){
                driveTrain.move(endPose.x, power);
                driveTrain.strafe(endPose.y, power);
                driveTrain.turn(endPose.orientation, power);
        }
        else if(chosenPath == Path.DIRECT){
            double turnAngle = Math.toDegrees(Math.atan2(endPose.x, endPose.y));
            driveTrain.turn((float)(-90 - turnAngle), power); //find correct angle

            double vectorMagnitude = Math.hypot(endPose.x, endPose.y);
            driveTrain.move(vectorMagnitude, power);
        }
    }
}