package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.navigation.Gyro;

/**
 * This op-mode runs the robot during the Driver Controlled Period.
 * It allows the robot to be driven, and for all the attachments to be controlled.
 */
@TeleOp(name = "Slides Test", group = "TEST")
public class SlidesTest extends OpMode{
    
    private ShivaRobot robot = new ShivaRobot();
    
    public void init() {
        // Initialize the robot interface
        robot.init(telemetry, hardwareMap);
    }

    public void loop() {
        slides();
        telemetry();
    }

    // Move the slides up and down and spin the intake motor
    private void slides() {
        robot.slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad2.right_stick_y > 0 /*&& robot.slides_motor.getCurrentPosition() <= 0*/) 
        {
            robot.slides_motor.setPower(gamepad2.right_stick_y);
        }
    
        else if(gamepad2.right_stick_y < 0 /*&& robot.slides_motor.getCurrentPosition() >= -3150*/) 
        {
            robot.slides_motor.setPower(gamepad2.right_stick_y);
        }
        else 
        {
            robot.slides_motor.setPower(0);
        }
    }

    // Display info on the driver station
    private void telemetry() {
        telemetry.addData("Running", "Loop");        
        telemetry.addData("Slide Motor", "Position: " + robot.slides_motor.getCurrentPosition());
    }
}
