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

/**
 * This op-mode tests the Alliance classes
 */
@Autonomous(name = "Alliance Test", group = "Test")
public class AllianceTest extends LinearOpMode {

  ShivaRobot robot = new ShivaRobot();
  ShivaAlliance alliance = new ShivaAlliance();

  /**
   * Repeatedly test and report the alliance detected by the ShivaAlliance class
   */
  @Override
  public void runOpMode() {
    // Initialize the robot interface
    robot.init(telemetry, hardwareMap);

    // Wait for the start button to be pressed.
    waitForStart();

    // Loop until we are asked to stop
    while (opModeIsActive()) {
      Alliance detectedAlliance = alliance.detectCurrent(robot);
      telemetry.addLine().addData("Alliance: ", detectedAlliance);
      telemetry.update();
    }
  }
}
