package org.firstinspires.ftc.teamcode.navigation;

public class Pose
{
    //Pose class stores an X position, Y position, and Orientation

    //Variables stored as Double and Integer to allow for a null value if necessary
    public double x;
    public double y;
    public Integer orientation; 

    public Pose(double x, double y, Integer orientation){
        this.x = x;
        this.y = y;
        this.orientation = orientation;
    }

    public void updatePose(double x, double y, Integer orientation){
        this.x = x;
        this.y = y;
        this.orientation = orientation;
    }
}