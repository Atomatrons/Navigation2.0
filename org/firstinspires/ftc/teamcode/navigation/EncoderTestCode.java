// /*package org.firstinspires.ftc.teamcode.navigation;

// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import org.firstinspires.ftc.robotcore.external.Telemetry;


// import org.firstinspires.ftc.teamcode.ShivaRobot;

// /**
//  * Provides low-level autonomous movement functionality for the robot
//  */
// @Autonomous(name = "EncoderTestCode", group = "Test")
// public class EncoderTestCode extends LinearOpMode {
//     private Gyro gyro = null;
//     private Telemetry telemetry = null;

//     // Cache the drivetrain motors in this class
//     private DcMotor front_left  = null;
//     private DcMotor front_right = null;
//     private DcMotor back_left   = null;
//     private DcMotor back_right  = null;

//     // Cache the deadwheel motors in this class
//     private DcMotor x_encoder = null;
//     private DcMotor y_encoder = null;

//     private ShivaRobot robot = new ShivaRobot();
    
//     @Override
//     public void runOpMode() {
//         // Save values as instance variables
//         telemetry = robot.telemetry;
//         front_left  = robot.front_left;
//         front_right = robot.front_right;
//         back_left   = robot.back_left;
//         back_right  = robot.back_right;
//         x_encoder = robot.x_encoder;
//         y_encoder = robot.y_encoder;
        
//         // Initialize the motors to run without encoders
//         //reset();
//         //Edited
//     double initalPositionY = y_encoder.getCurrentPosition();
//     double currentPositionY = initalPositionY;
//     telemetry.add("Status", "Moving Forward");
//     telemetry.update();
//     forward();
//         while(currentPositionY <= initalPositionY + 100 ) {
//             currentPosition = y_encoder.getCurrentPosition();
//             telemetry.add("Position", currentPositionY);
//             telemetry.update();
//         }
//     stop();
//     telemetry.add("Status", "Moved Forward. Stopping");
//     telemetry.add("Initial Position", initalPositionY);
//     telemetry.add("Final Posistion", currentPositionY);
//     telemetry.update();
//     sleep(2500);

//     initalPositionY = y_encoder.getCurrentPosition();
//     currentPositionY = initalPositionY;
//     telemetry.add("Status", "Moving Backward");
//     telemetry.update()
//     backward();
//         while(currentPositionY >= initalPositionY - 100) {
//             currentPosition = y_encoder.getCurrentPosition();
//             telemetry.add("Position", currentPositionY);
//             telemetry.update();
//         }
//     stop();
//     telemetry.add("Status", "Moved Backwards. Stopping");
//     telemetry.add("Initial Position", initalPositionY);
//     telemetry.add("Final Posistion", currentPositionY);
//     telemetry.update();

//     }
   
//     //Methods
//     private void stop() {
//         front_left.setPower(0);
//         front_right.setPower(0);
//         back_left.setPower(0);
//         back_right.setPower(0);
//     } 

//     private void forward() {
//         front_left.setPower(0.5);
//         front_right.setPower(-0.5);
//         back_left.setPower(0.5);
//         back_right.setPower(-0.5);
//     }

//     private void backward() {
//         front_left.setPower(-0.5);
//         front_right.setPower(0.5);
//         back_left.setPower(-0.5);
//         back_right.setPower(0.5);
//     }

//     private void strafeLeft() {
//         front_left.setPower(-0.5);
//         front_right.setPower(-0.5);
//          back_left.setPower(0.5);
//         back_right.setPower(0.5);
//     }

//     private void strafeRight() {
//         front_left.setPower(0.5);
//         front_right.setPower(0.5);
//         back_left.setPower(-0.5);
//         back_right.setPower(-0.5);
//     }
// } */
