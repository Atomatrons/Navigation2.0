package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ShivaRobot;

/**
 * Provides low-level autonomous movement functionality for the robot
 */
public class DriveTrain {
    private Gyro gyro = null;

    // Cache the drivetrain motors in this class
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    
    private static final float AMPLIFIER = 0.02f;
    
    public void init(ShivaRobot robot, Gyro newGyro)
    {
        // Save values as instance variables
        gyro = newGyro;
        front_left  = robot.front_left;
        front_right = robot.front_right;
        back_left   = robot.back_left;
        back_right  = robot.back_right;
        
        // Initialize the motors to run using encoders
        reset();
    }
    
    // Move the robot forward the specified rotations, at the specified power
    // Power must be a value from -1.0 to 1.0
    // The gyro is used to help the robot drive straight
    public void forward(double rotationsToSpin, double power)
    {
        // Get the robot's current heading, and compute the number of ticks needed to move
        float startAngle = (float)gyro.getCurrentAngle();
        
        // Set the wheels so we move forwards
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Start the robot moving.
        //  The robot will stop moving when the wheels turn the specified number of rotations
        startMovement(rotationsToSpin, power);
                
        // While the robot is moving, use the gyro to help it move in a stright line
        while(front_left.isBusy())
        {
            adjust(-getCorrectionAngle(startAngle), (float)power);
        }
        
        // Reset the tick count on all the wheels
        reset();
    }
    
    // Move the robot backwards the specified rotations, at the specified power
    // Power must be a value from -1.0 to 1.0
    // The gyro is used to help the robot drive straight
    public void backward(double rotationsToSpin, double power)
    {   
        float startAngle = (float)gyro.getCurrentAngle();
        int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin);
        
        // Set the wheels so we move backwards
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        startMovement(rotationsToSpin, power);

        while(back_left.isBusy())
        {
            adjust(getCorrectionAngle(startAngle), (float)power);
        }
        
        reset();
    }

    // Move the robot sideways to the right the specified rotations, at the specified power
    // Power must be a value from -1.0 to 1.0
    // The gyro is NOT used to help the robot drive straight
    public void strafeRight(double rotationsToSpin, double power) throws InterruptedException
    {
        int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin);

        // Set the wheels so we move to the right
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        
        startMovement(rotationsToSpin, power);
        
        // Wait for the robot to reach its destination
        while(front_left.isBusy())
        {
                
        }

        reset();
    }
    
    // Move the robot sideways to the left the specified rotations, at the specified power
    // Power must be a value from -1.0 to 1.0
    // The gyro is NOT used to help the robot drive straight
    public void strafeLeft(double rotationsToSpin, double power) throws InterruptedException
    {
        int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin);
                
        // Set the wheels so we move to the left
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        
        startMovement(rotationsToSpin, power);
        
        // Wait for the robot to reach its destination
        while(front_left.isBusy())
        {
                
        }

        reset();
    }
    
    // Turn the robot to the specified compass point, using a spin turn
    // compassPoint must be a value from -180.0 to 180.0
    // power must be a value from -1.0 to 1.0
    public void turn(float compassPoint, float power)
    {
        // Turns are made based on the gyro instead of based on encoders
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        if (gyro.isClockwise(compassPoint))
        {
            front_left.setDirection(DcMotorSimple.Direction.FORWARD);
            back_left.setDirection(DcMotorSimple.Direction.FORWARD);
            front_right.setDirection(DcMotorSimple.Direction.FORWARD);
            back_right.setDirection(DcMotorSimple.Direction.FORWARD);
                
            front_left.setPower(power);
            back_left.setPower(power);
            front_right.setPower(power);
            back_right.setPower(power);
            while(gyro.getCurrentAngle() < compassPoint)
            {
                
            }
        }
        else if (!gyro.isClockwise(compassPoint))
        {
            front_left.setDirection(DcMotorSimple.Direction.REVERSE);
            back_left.setDirection(DcMotorSimple.Direction.REVERSE);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
            back_right.setDirection(DcMotorSimple.Direction.REVERSE);
                
            front_left.setPower(power);
            back_left.setPower(power);
            front_right.setPower(power);
            back_right.setPower(power);
            while(gyro.getCurrentAngle() > compassPoint)
            {
                
            }
        }
        
        reset();
        
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void stop()
    {
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }
    
    public void reset()
    {
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Move the robot the specified rotations, at the specified power
    // Power must be a value from -1.0 to 1.0
    // The direction to move must be set before calling this method
    private void startMovement(double rotationsToSpin, double power) {
        // Compute how many ticks the wheels need to spin
        int target_position = (int) Math.round(ShivaRobot.MOTOR_TICKS_PER_360 * rotationsToSpin);

        // Set the target position
        front_left.setTargetPosition(target_position);
        back_left.setTargetPosition(target_position);
        front_right.setTargetPosition(target_position);
        back_right.setTargetPosition(target_position);
        
        // The wheels will move until they reach the specified target ticks
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Start the robot moving
        front_left.setPower((float)power);
        back_left.setPower((float)power);
        front_right.setPower((float)power);
        back_right.setPower((float)power);
    }

    
    // Adjust the power to the wheels, based on the current gyro reading, in case the robot drifts off course
    private void adjust(float angle, float basePower)
    {
        float powerChange = Math.abs(angle * AMPLIFIER);
        
        if(angle > 0)
        {
            front_left.setPower(basePower - powerChange);
            back_left.setPower(basePower - powerChange);
            front_right.setPower(basePower + powerChange);
            back_right.setPower(basePower + powerChange);
        }
        else if(angle < 0)
        {
            front_left.setPower(basePower + powerChange);
            back_left.setPower(basePower + powerChange);
            front_right.setPower(basePower - powerChange);
            back_right.setPower(basePower - powerChange);
        }
    }
    
    public float getCorrectionAngle(float startAngle)
    {
        float correctionAngle = startAngle - (float)gyro.getCurrentAngle();
        return correctionAngle;
    }
}