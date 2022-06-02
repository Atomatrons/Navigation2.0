package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ShivaRobot;

public class GPS{
    
    private ShivaRobot robot;
    private Gyro gyro;

    public void init(ShivaRobot robot, Gyro gyro){
        this.robot = robot;
        this.gyro = gyro;
    }

    public Pose getCurrentPose(){
        return new Pose((double)robot.x_encoder.getCurrentPosition(), 
            (double)robot.y_encoder.getCurrentPosition(), (int)gyro.getCurrentAngle());
    }
}