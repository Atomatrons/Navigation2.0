package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.teamcode.navigation.DriveTrain;
import org.firstinspires.ftc.teamcode.navigation.Gyro;

/**
 * This op-mode tests the Alliance classes
 */
@Autonomous(name = "DriveTrain Test", group = "Test")
public class DriveTrainTest extends LinearOpMode {

  ShivaRobot robot = new ShivaRobot();
  Gyro gyro = new Gyro();
  DriveTrain drivetrain = new DriveTrain();

  /**
   * Repeatedly test and report the alliance detected by the ShivaAlliance class
   */
  @Override
  public void runOpMode() throws InterruptedException{
    // Initialize the robot interface
    robot.init(telemetry, hardwareMap);
    gyro.init(robot);
    drivetrain.init(robot, gyro);

    // Wait for the start button to be pressed.
    waitForStart();

    // Drive the robot a short distance forward, backward, left, and right, and then turn 180 in each direction.

    drivetrain.move (5, 0.5);
    drivetrain.strafe (3, 0.5);

    while (opModeIsActive()) {


    }
  }
}
