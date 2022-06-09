package org.firstinspires.ftc.teamcode;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.DriveTrain;
import org.firstinspires.ftc.teamcode.navigation.Gyro;
import org.firstinspires.ftc.teamcode.navigation.GPS;


/**
 * This op-mode tests the Alliance classes
 */
@Autonomous(name = "GPS Test", group = "Test")
public class GPSTest extends LinearOpMode {

    ShivaRobot robot = new ShivaRobot();
    Gyro gyro = new Gyro();
    //DriveTrain drivetrain = new DriveTrain();
    GPS gps = new GPS();

    /**
     * Test GPS class and x/y encoders.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot interface
        robot.init(telemetry, hardwareMap);
        gyro.init(robot);
        gyro.quietMode = true;      // turn off telemetry
        gps.init(robot, gyro);
        //drivetrain.init(robot, gyro);

        //robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.back_right.setTargetPosition(4190 * 2);

        // Wait for the start button to be pressed.
        waitForStart();


        //while(robot.back_right.isBusy()){

        //}

        // test_setPowerAndSleep();
        test_setPowerAndUseDeadwheel();

        stopRobot();
        printData();

        //Drive the robot 4 rotations forward
        while (opModeIsActive()) {
        }
    }

    private void test_setPowerAndUseDeadwheel() {
        setZeroPowerBehavior();
        displayZeroPowerBehavior();

        robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power = 1.0;
        robot.front_left.setPower(power);
        robot.back_left.setPower(power);
        robot.front_right.setPower(power);
        robot.back_right.setPower(power);

        int count = 0;
        double startTime = this.time;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(gps.getCurrentPose().y < 5000){
            count++;
        }
        double endTime = timer.milliseconds();

        double msPerLoop = endTime / count;
        telemetry.addData("msPerLoop", msPerLoop);
        // 7.8ms including gyro, 3.7ms not including gyro, 2.4ms doing just y encoder
        // 1 encoder: stops ~ 3060
        // 2 encoders: stops at 3008 - 3100
        // 2 enc + gyro: stops at 3062, 3091, 3050, 3087
    }

    private void test_setPowerAndSleep() throws InterruptedException {
        double power = 1.0;
        robot.front_left.setPower(power);
        robot.back_left.setPower(power);
        robot.front_right.setPower(power);
        robot.back_right.setPower(power);

        Thread.sleep(3000);
    }

    private void stopRobot() {
        robot.front_left.setPower(0);
        robot.back_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_right.setPower(0);
    }

    private void printData() {
        telemetry.addData("GPS: X Encoder Position", gps.getCurrentPose().x);
        telemetry.addData("GPS: Y Encoder Position", gps.getCurrentPose().y);
        telemetry.addData("GPS: Orientation", gps.getCurrentPose().orientation);
        telemetry.update();
    }

    private void setZeroPowerBehavior() {
        DcMotor.ZeroPowerBehavior zpb = DcMotor.ZeroPowerBehavior.BRAKE;
        robot.front_left.setZeroPowerBehavior(zpb);
        robot.back_left.setZeroPowerBehavior(zpb);
        robot.front_right.setZeroPowerBehavior(zpb);
        robot.back_right.setZeroPowerBehavior(zpb);
    }

    private void displayZeroPowerBehavior() {
        DcMotor.ZeroPowerBehavior zpb;
        zpb = robot.front_left.getZeroPowerBehavior();
        telemetry.addData("FL zpb", getZeroPowerBehaviorName(zpb));
        zpb = robot.back_left.getZeroPowerBehavior();
        telemetry.addData("FL zpb", getZeroPowerBehaviorName(zpb));
        zpb = robot.front_right.getZeroPowerBehavior();
        telemetry.addData("FL zpb", getZeroPowerBehaviorName(zpb));
        zpb = robot.back_right.getZeroPowerBehavior();
        telemetry.addData("FL zpb", getZeroPowerBehaviorName(zpb));
    }

    private String getZeroPowerBehaviorName(DcMotor.ZeroPowerBehavior zpb) {
        switch (zpb) {
            case BRAKE:
                return "BRAKE";
            case FLOAT:
                return "FLOAT";
            case UNKNOWN:
                return "UNKNOWN";
            default:
                return "INVALID";
        }
    }
}
