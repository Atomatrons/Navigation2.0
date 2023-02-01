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

/**
 * This op-mode tests the Gyro class
 */
@Autonomous(name = "Gyro Test 2", group = "Test")
public class GyroTestII extends LinearOpMode {

	ShivaRobot robot = new ShivaRobot();
	Gyro gyro = new Gyro();

	/**
	 * Repeatedly test and report the angles detected by the Gyro
	 */
	@Override
	public void runOpMode() {
		// Initialize the robot interface
		robot.init(telemetry, hardwareMap);
		gyro.init(robot);
		telemetry.addData("GyroTest", "Robot Initialized v1.1");
		telemetry.update();

		// Wait for the start button to be pressed.
		waitForStart();
		telemetry.addData("GyroTest", "Robot Started");
		telemetry.update();

		// Loop until we are asked to stop, displaying the gyro info in telemetry
		while (opModeIsActive()) {
			telemetry.addData("GyroTest", "Robot Looping");
			gyro.getCurrentAngle();
			this.telemetry.addData("POS ", "1");
			if (gyro.isRobotTipping())	{
				this.telemetry.addData("Mayday ", "Tipping");
        	} else {
           		this.telemetry.addData("Status ", "Gud");
			}
			this.telemetry.addData("POS 2", "2");
			telemetry.update();
		}
	}
}
