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
@TeleOp(name = "DriveHardSSSSS", group = "Production")
public class DriveHardSquaredHarder extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private ShivaRobot robot = new ShivaRobot();
    private Gyro gyro = new Gyro();

    private final int MAX_SLIDES_POSITION = 4294;
    private final int MIN_SLIDES_POSITION = 0;

    private boolean slidesAreMoving = false;
    private boolean fieldOriendtedDrive = true;
    
    public void init() {
        // Initialize the robot interface
        robot.init(telemetry, hardwareMap);
        gyro.init(robot);
        gyro.quietMode = true;
        robot.slides_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.grip_servo.scaleRange(0.45, 1);
    }

    public void loop() {
        drive();
        slides();
        grip();
        boolean isTipping = gyro.isRobotTipping();
        if (gyro.isRobotTipping()) {
            stopTipping();
        }

        if (gamepad1.y) {
            fieldOriendtedDrive = !fieldOriendtedDrive;
            if (fieldOriendtedDrive) {
                gyro.resetAngles();
            }
        }

        telemetry();
        telemetry.update();
    }

    // Move the robot
    private void drive() {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = (gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y)) * 0.7;
        double twist = -(gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x)) * 0.55;
        double strafe = -gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x)* 0.7;
        
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
            strafe = 0.5;
        }
        if(gamepad1.dpad_right)
        {
            strafe = -0.5;

        }
        double [] speeds = {0, 0, 0, 0};

        if (fieldOriendtedDrive) {
        double [] fielding = rotateVector(drive, strafe, -gyro.getCurrentAngle());
        drive = fielding[0];
        strafe = fielding[1];
        }

        if(slidesAreMoving){
            speeds = new double []{
                -(drive + strafe + twist) / 2, //Front left power
                -(drive - strafe - twist) / 2, //Front right power
                -(drive - strafe + twist) / 2, //Back left power
                -(drive + strafe - twist) / 2 //Back right power
            };
        }
        else{
            speeds = new double []{
                -(drive + strafe + twist), //Front left power
                -(drive - strafe - twist), //Front right power
                -(drive - strafe + twist), //Back left power
                -(drive + strafe - twist) //Back right power
            };
        }



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

    public double[] rotateVector(double x, double y, double angle) {
		double cosA = Math.cos(angle * (Math.PI / 180.0));
		double sinA = Math.sin(angle * (Math.PI / 180.0));
		double[] out = new double[2];
		out[0] = x * cosA - y * sinA;
		out[1] = x * sinA + y * cosA;
		return out;
	}

    // Move the slides up and down
    private void slides() {
        robot.slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad2.left_stick_y > 0  && robot.slides_motor.getCurrentPosition() <= MIN_SLIDES_POSITION) 
        {
            robot.slides_motor.setPower(gamepad2.left_stick_y*Math.abs(gamepad2.left_stick_y));
            slidesAreMoving = true; 
        }
    
        else if(gamepad2.left_stick_y < 0  &&  robot.slides_motor.getCurrentPosition()  >= -MAX_SLIDES_POSITION) 
        {
            robot.slides_motor.setPower(gamepad2.left_stick_y*Math.abs(gamepad2.left_stick_y));
            slidesAreMoving = true;
        }
        else if(gamepad2.dpad_up)
        {
            robot.slides_motor.setPower(0.5);
            slidesAreMoving = true;
        }
        else if(gamepad2.dpad_down)
        {
            robot.slides_motor.setPower(0.5);
            slidesAreMoving = true; 
        }    
        else 
        {
            robot.slides_motor.setPower(0);
            slidesAreMoving = false;
        }
    }

    //Move servo to grip cones
    private void grip(){
        if(gamepad2.a){
            robot.grip_servo.setPosition(1);
        }
        if(gamepad2.b){
            robot.grip_servo.setPosition(0);
        }
    }

    // Display info on the driver station
    private void telemetry() {
        telemetry.addData("Running", "Loop");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        
        telemetry.addData("Slide Motor", "Position: " + robot.slides_motor.getCurrentPosition());
        telemetry.addData("Grip Servo", "Position: " + robot.grip_servo.getPosition());
        telemetry.addData("Test Encoder", "Ticks: " + robot.x_encoder.getCurrentPosition());
    }

    private void stopTipping () {
        telemetry.addData("Maverick ", "Deploying Counter-Measures");
    }


}
