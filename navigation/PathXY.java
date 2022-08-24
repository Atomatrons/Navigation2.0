package org.firstinspires.ftc.teamcode.navigation;

public class PathXY extends Path{
    @Override
    public void move(){
        super.move();

        driveTrain.strafeRight(endPose.x, 0.8);
        driveTrain.forward(endPose.y, 0.8);
        driveTrain.turn(endPose.orientation, 0.8);
    }
}