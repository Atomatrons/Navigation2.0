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
import org.firstinspires.ftc.teamcode.navigation.Navigation; 
import org.firstinspires.ftc.teamcode.navigation.Path; 
import org.firstinspires.ftc.teamcode.navigation.DriveTrain;
import org.firstinspires.ftc.teamcode.navigation.Gyro;
import org.firstinspires.ftc.teamcode.navigation.Pose;


/**
 * This op-mode tests the Alliance classes
 */
@Autonomous(name = "Navigation", group = "Test")
public class NavigationTest extends LinearOpMode {

  ShivaRobot robot = new ShivaRobot();
  Gyro gyro = new Gyro();
  DriveTrain driveTrain = new DriveTrain();
  Navigation nav = new Navigation();

  /**
   * Repeatedly test and report the alliance detected by the ShivaAlliance class
   */
  @Override
  public void runOpMode() throws InterruptedException{
    // Initialize the robot interface
    robot.init(telemetry, hardwareMap);
    gyro.init(robot);
    driveTrain.init(robot, gyro);
    nav.init(driveTrain);

    // Wait for the start button to be pressed.
    waitForStart();

    Pose endPose = new Pose(4, 5, 60);

    nav.goToPose(endPose, Path.DIRECT, 0.8);

    while (opModeIsActive()) {

    }
  }
}
