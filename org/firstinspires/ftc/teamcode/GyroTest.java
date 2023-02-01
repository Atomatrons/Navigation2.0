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
@Autonomous(name = "Gyro Test X", group = "Test")
public class GyroTest extends LinearOpMode {

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
		telemetry.addData("GyroTest", "Robot Initialized v1.0");
		telemetry.update();

		// Wait for the start button to be pressed.
		waitForStart();
		telemetry.addData("GyroTest", "Robot Started");
		telemetry.update();

		// Loop until we are asked to stop, displaying the gyro info in telemetry
		while (opModeIsActive()) {
			gyro.getCurrentAngle();
			if (gyro.isRobotTipping())	{
				this.telemetry.addData("Mayday ", "Tipping");
        }else{
            this.telemetry.addData("Status ", "Gud");
		}
			telemetry.update();
		}
	}
}
