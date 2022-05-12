package org.firstinspires.ftc.teamcode.navigation; 

public class Navigation {
    //Navigation class tells the Robot to move to a specific Pose, using a specific Path

    private Pose startPose = null;
    
    public void init(Pose currentPose){
        setStartPose(currentPose);
    }

    public void goToPose(Pose endPose, Path chosenPath){
        //To be implemented
    }

    //Pass in currentPose from OpMode
    public void setStartPose(Pose currentPose){
        startPose = currentPose;
    }
}