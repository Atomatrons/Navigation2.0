package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is used to define all the specific hardware for our robot.
 * All other code can use this class to refer to the robot hardware as software objects.
 */
public class ShivaRobot {

    // Wheel motors
    public DcMotor front_left = null;
    public DcMotor front_right = null;
    public DcMotor back_left = null;
    public DcMotor back_right = null;

    // Dead wheel encoders
    public DcMotor x_encoder = null;
    public DcMotor y_encoder = null;

    // Slides motors
    public DcMotor slides_motor = null;
    public DcMotor intake_spinner = null;

    // Gyro
    public BNO055IMU imu = null;

    // Alliance Sensor
    public NormalizedColorSensor allianceColorSensor = null;

    // Constants
    public static final double MOTOR_TICKS_PER_360 = 537.7;
    public static final double DEAD_WHEEL_TICKS = 4190;

    // For doing execution time measurements
    private ElapsedTime period = new ElapsedTime();

    // To allow non OpMode classes to display info to telemetry:
    public Telemetry telemetry;

    /* Constructor */
    public ShivaRobot() {}

    /* Initialize standard Hardware interfaces */
    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        
        // Wheel motors
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");

        // Set Motors to not use encoders
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         // Dead wheel encoders; set current position to 0,0
         x_encoder = back_left;
         y_encoder = front_left;

        // Gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Alliance Sensor
        // It's recommended to use NormalizedColorSensor over ColorSensor, because
        // NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        //allianceColorSensor = hardwareMap.get(NormalizedColorSensor.class, "alliance_sensor");

        // Reverse the direction of the right side motors,
        // so power with the same sing (+ or -) causes the robot to move in the same direction,
        // no matter which wheels the power is applied to.
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
    }
}
