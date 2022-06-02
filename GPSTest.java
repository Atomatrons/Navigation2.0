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
  public void runOpMode() throws InterruptedException{
    // Initialize the robot interface
    robot.init(telemetry, hardwareMap);
    gyro.init(robot);
    gps.init(robot, gyro);
    //drivetrain.init(robot, gyro);

    //robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //robot.back_right.setTargetPosition(4190 * 2);

    // Wait for the start button to be pressed.
    waitForStart();

    //Drive the robot 4 rotations forward
    while (opModeIsActive()) {

      robot.front_left.setPower(0.5);
      robot.back_left.setPower(0.5);
      robot.front_right.setPower(0.5);
      robot.back_right.setPower(0.5);
      
      //while(robot.back_right.isBusy()){

      //}

      Thread.sleep(5000);

      robot.front_left.setPower(0);
      robot.back_left.setPower(0);
      robot.front_right.setPower(0);
      robot.back_right.setPower(0);


      telemetry.addData("GPS: X Encoder Position", gps.getCurrentPose().x);
      telemetry.addData("GPS: Y Encoder Position", gps.getCurrentPose().y);
      telemetry.addData("GPS: Orientation", gps.getCurrentPose().orientation);
    }
  }
}
