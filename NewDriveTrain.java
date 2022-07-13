package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.navigation.DriveTrain;
import org.firstinspires.ftc.teamcode.navigation.Gyro;

@Autonomous(name = "New Drive Train", group = "Test")
public class NewDriveTrain extends LinearOpMode {

  private DcMotor front_right;
  private DcMotor back_right;
  private DcMotor front_left;
  private DcMotor back_left;

  private ShivaRobot robot;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double speed;
    double rotations;

    robot = new ShivaRobot();

    robot.init(telemetry, hardwareMap);

    front_right = robot.front_right;
    back_right = robot.back_right;
    front_left = robot.front_left;
    back_left = robot.back_left;

    // Put initialization blocks here.
    waitForStart();

    //front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    //back_right.setDirection(DcMotorSimple.Direction.REVERSE);

    if (opModeIsActive()) {
      // Put run blocks here.
      telemetry.addData("front_left position (y)", robot.y_encoder.getCurrentPosition());
      telemetry.addData("back_left position (x)", robot.x_encoder.getCurrentPosition());
      telemetry.addData("front_right", front_right.getCurrentPosition());
      telemetry.addData("back_right", back_right.getCurrentPosition());
      telemetry.update();

      speed = 0.5;
      rotations = 2;

      front_right.setPower(speed);
      back_right.setPower(speed);
      front_left.setPower(speed);
      back_left.setPower(speed);

      int initalPosition = robot.y_encoder.getCurrentPosition();

      while(robot.y_encoder.getCurrentPosition() <= initalPosition + (rotations * ShivaRobot.DEAD_WHEEL_TICKS)){

      }

      front_right.setPower(0);
      back_right.setPower(0);
      front_left.setPower(0);
      back_left.setPower(0);
      
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.addData("front_left position (y)", front_left.getCurrentPosition());
        telemetry.addData("back_left position (x)", back_left.getCurrentPosition());
        telemetry.addData("front_right", front_right.getCurrentPosition());
        telemetry.addData("back_right", back_right.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}