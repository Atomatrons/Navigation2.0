package org.firstinspires.ftc.teamcode.navigation;

public class Pose
{
    //Pose class stores an X position, Y position, and Orientation

    //Variables stored as Double and Integer to allow for a null value if necessary
    public Double x;
    public Double y;
    public Integer orientation; 

    public Pose(Double x, Double y, Integer orientation){
        this.x = x;
        this.y = y;
        this.orientation = orientation;
    }

    public void updatePose(Double x, Double y, Integer orientation){
        this.x = x;
        this.y = y;
        this.orientation = orientation;
    }
}