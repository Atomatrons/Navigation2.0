package org.firstinspires.ftc.teamcode.navigation;

public class PathXY extends Path{
    @Override
    public void move(){
        super.move();

        driveTrain.strafeRight(endPose.X, 0.8);
        drivetrain.forward(endPose.y, 0.8);
        driveTrain.turn(endPose.orientation, 0.8);
    }
}