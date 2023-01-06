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

import org.firstinspires.ftc.teamcode.navigation.Gyro;
import org.firstinspires.ftc.teamcode.utility.Distance;

/**
 * This autonomous op-mode tests the Distance class
 */
@Autonomous(name = "Distance Test VS Code", group = "Test")
public class DistanceTest extends LinearOpMode {

	ShivaRobot robot = new ShivaRobot();
	Distance distance = new Distance();

	/**
	 * Repeatedly test and report the angles detected by the Gyro
	 */
	@Override
	public void runOpMode() {
		// Initialize the robot interface
		robot.init(telemetry, hardwareMap);
		distance.init(robot);
		telemetry.addData("DistanceTest", "Robot Initialized v1.0");
		telemetry.update();

		// Wait for the start button to be pressed.
		waitForStart();
		telemetry.addData("DistanceTest", "Robot Started");
		telemetry.update();

		// Loop until we are asked to stop, displaying the gyro info in telemetry
		while (opModeIsActive()) {
			boolean targetLocked = distance.targetLocked();
			double range = distance.distance(true);
			telemetry.addData("Is Target Locked", targetLocked);
			telemetry.addData("Distance", range);
			telemetry.update();
		}
	}
}