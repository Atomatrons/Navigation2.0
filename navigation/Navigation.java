package org.firstinspires.ftc.teamcode.navigation; 

public class Navigation {
    //Navigation class tells the Robot to move to a specific Pose, using a specific Path

    private Pose startPose = null;
    private DriveTrain driveTrain = null;
    
    public void init(Pose currentPose, DriveTrain driveTrain){
        setStartPose(currentPose);
        this.driveTrain = drivetrain;
    }

    public void goToPose(Pose endPose, Path chosenPath){
        chosenPath.endPose = endPose;
        chosenPath.driveTrain = driveTrain;

        chosenPath.move(); 
    }

    //Pass in currentPose from OpMode
    public void setStartPose(Pose currentPose){
        startPose = currentPose;
    }
}