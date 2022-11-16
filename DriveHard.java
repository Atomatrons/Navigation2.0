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
@TeleOp(name = "DriveHard", group = "Production")
public class DriveHard extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private ShivaRobot robot = new ShivaRobot();
    private Gyro gyro = new Gyro();

    private final int MAX_SLIDES_POSITION = 3150;
    private final int MIN_SLIDES_POSITION = 0;
    
    public void init() {
        // Initialize the robot interface
        robot.init(telemetry, hardwareMap);
        gyro.init(robot);
        gyro.quietMode = false;
    }

    public void loop() {
        drive();
        slides();
        grip();
        boolean isTipping = gyro.isRobotTipping();
        if (gyro.isRobotTipping()) {
            stopTipping();
        }

        //telemetry();
        telemetry.update();
    }

    // Move the robot
    private void drive() {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y;
        double twist = -gamepad1.right_stick_x / 2;
        double strafe = -gamepad1.left_stick_x / 2;
        
        if(gamepad1.dpad_up)
        {
            drive = -0.2;
        }
        if(gamepad1.dpad_down)
        {
            drive = 0.2;
        }
        if(gamepad1.dpad_left)
        {
            strafe = 0.4;
        }
        if(gamepad1.dpad_right)
        {
            strafe = -0.4;

        }

        double[] speeds = {
            -(drive + strafe + twist), //Front left power
            -(drive - strafe - twist), //Front right power
            -(drive - strafe + twist), //Back left power
            -(drive + strafe - twist) //Back right power
        };


        // Normalizes values
        double max = Math.abs(speeds[0]);
        for(int i = 1; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        
        // apply the calculated values to the motors.
        robot.front_left.setPower(speeds[0]);
        robot.front_right.setPower(speeds[1]);
        robot.back_left.setPower(speeds[2]);
        robot.back_right.setPower(speeds[3]);
        
        robot.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Move the slides up and down
    private void slides() {
        robot.slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad2.left_stick_y > 0 && robot.slides_motor.getCurrentPosition() <= MIN_SLIDES_POSITION) 
        {
            robot.slides_motor.setPower(gamepad2.left_stick_y);
        }
    
        else if(gamepad2.left_stick_y < 0 && robot.slides_motor.getCurrentPosition() >= -MAX_SLIDES_POSITION) 
        {
            robot.slides_motor.setPower(gamepad2.left_stick_y);
        }
        else 
        {
            robot.slides_motor.setPower(0);
        }
    }

    //Move servo to grip cones
    private void grip(){
        robot.grip_servo.setPosition(gamepad2.right_trigger);
    }

    // Display info on the driver station
    private void telemetry() {
        telemetry.addData("Running", "Loop");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        
        //telemetry.addData("Slide Motor", "Position: " + robot.slides_motor.getCurrentPosition());
        telemetry.addData("Test Encoder", "Ticks: " + robot.x_encoder.getCurrentPosition());
    }

    private void stopTipping () {
        telemetry.addData("Maverick ", "Deploying Counter-Measures");
    }
}
