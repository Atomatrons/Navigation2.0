package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.teamcode.ShivaRobot;

/**
 * Provides low-level autonomous movement functionality for the robot
 */
public class DriveTrain {
    private Gyro gyro = null;
    private Telemetry telemetry = null;

    // Cache the drivetrain motors in this class
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    // Cache the deadwheel motors in this class
    private DcMotor x_encoder = null;
    private DcMotor y_encoder = null;
    
    {
        // Save values as instance variables
        telemetry = robot.telemetry;
        front_left  = robot.front_left;
        front_right = robot.front_right;
        back_left   = robot.back_left;
        back_right  = robot.back_right;
        x_encoder = robot.x_encoder;
        y_encoder = robot.y_encoder;
        
        // Initialize the motors to run without encoders
        //reset();
    }

        //Edited
        double initalPosition = y_encoder.getCurrentPosition();
        double currentPosition = initalPosition;
        telemetry.add("Status", "Stating To Move.");
        telemetry.update();
        forward();
        while(currentPosition <= initalPosition + 100 ) {
            currentPosition = y_encoder.getCurrentPosition();
            telemetry.add("Position", currentPosition);
            telemetry.update();
        }
    stop();
    telemetry.add("Status", "Mission Complete. Stopping");
    telemetry.add("Initial Position", initalPosition);
    telemetry.add("Final Posistion", currentPosition);
    telemetry.update();
    
    //Methods
    private void stop() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0)
    }

    private void forward() {
        front_left.setPower(0.5);
        front_right.setPower(-0.5);
        back_left.setPower(0.5);
        back_right.setPower(-0.5);
    }
}