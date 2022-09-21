package org.firstinspires.ftc.teamcode

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.navigation.DriveTrain;
import org.firstinspires.ftc.teamcode.navigation.Gyro;

/**
 * This op-mode tests the Alliance classes
 */
@Autonomous(name = "Aarav_Owen_Test_Op", group = "Test")
public class TestOpModeAllDirections extends LinearOpMode {
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor slides;

    @Override
    public void runOpMode() {
      front_left = hardwareMap.get(DcMotor.class, "front_left");
      front_right = hardwareMap.get(DcMotor.class, "front_right");
      back_left = hardwareMap.get(DcMotor.class, "back_left");
      back_right = hardwareMap.get(DcMotor.class, "back_right");
      slides = hardwareMap.get(DcMotor.class, "slides");
                  
      waitForStart();
      // Linear Slides go up
      telemetry.addData("Linear Sildes Testing", "Up");
      telemetry.update();
      slides.setPower(-0.3);
      sleep(1000);
      // Linear Slides stop moving
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      slides.setPower(0);
      sleep(1000);
      // Linear Slides go down
      telemetry.addData("Linear Slides Testing", "Down");
      telemetry.update();
      slides.setPower(0.3);
      // Linear Slides stop moving
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      sleep(1000);
      slides.setPower(0);
      // Goes Forward
      sleep(1000);
      telemetry.addData("Testing", "Forward");
      telemetry.update();
      front_left.setPower(0.5);
      front_right.setPower(-0.5);
      back_left.setPower(0.5);
      back_right.setPower(-0.5);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      //Goes Backwards
      sleep(1000);
      telemetry.addData("Testing", "Backward");
      telemetry.update();
      front_left.setPower(-0.5);
      front_right.setPower(0.5);
      back_left.setPower(-0.5);
      back_right.setPower(0.5);
      //Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      sleep(1000);
      telemetry.addData("Testing", "Strafe Left");
      telemetry.update();
      //Goes Left
      front_left.setPower(-0.5);
      front_right.setPower(-0.5);
      back_left.setPower(0.5);
      back_right.setPower(0.5);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      //Goes Right
      sleep(1000);
      telemetry.addData("Testing", "Strafe Right");
      telemetry.update();
      front_left.setPower(0.5);
      front_right.setPower(0.5);
      back_left.setPower(-0.5);
      back_right.setPower(-0.5);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      //Goes Diagonally Forward Left
      sleep(1000);
      telemetry.addData("Testing Advanced", "Diagonally Forward Left");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(-0.5);
      back_left.setPower(0.5);
      back_right.setPower(0);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      // Goes Diagonally Backwards Right
      sleep(1000);
      telemetry.addData("Testing Advanced", "Diagonally Backwards Right");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0.5);
      back_left.setPower(-0.5);
      back_right.setPower(0);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      // Goes Diagonally Forwards Right
      sleep(1000);
      telemetry.addData("Testing Advanced", "Diagonally Forwards Right");
      telemetry.update();
      front_left.setPower(0.5);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(-0.5);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      // Goes Diagonally Backwards Left
      sleep(1000);
      telemetry.addData("Testing Advanced", "Diagonally Backwards Left");
      telemetry.update();
      front_left.setPower(-0.5);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0.5);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      // Spins Counter-Clockwise
      sleep(1000);
      telemetry.addData("Testing Advanced+", "Spin Counter-Clockwise");
      telemetry.update();
      front_left.setPower(-0.5);
      front_right.setPower(-0.5);
      back_left.setPower(-0.5);
      back_right.setPower(-0.5);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      // Spins Clockwise
      sleep(1000);
      telemetry.addData("Testing Advanced+", "Spin Clockwise");
      telemetry.update();
      front_left.setPower(0.5);
      front_right.setPower(0.5);
      back_left.setPower(0.5);
      back_right.setPower(0.5);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
    }
}  