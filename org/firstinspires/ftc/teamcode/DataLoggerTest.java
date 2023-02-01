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
import org.firstinspires.ftc.teamcode.utility.*;

import org.firstinspires.ftc.teamcode.navigation.Gyro;

/**
 * This op-mode tests the DataLogger class
 */
@Autonomous(name = "DataLogger Test", group = "Test")
public class DataLoggerTest extends LinearOpMode {

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
		telemetry.addData("DataLoggerTest", "Robot Initialized");
		telemetry.update();

		// Test the data logger
		// Create the loggers
		DataLogger opmodeLogger = new DataLogger("DataLoggerTest", robot.telemetry, true);
		DataLogger gyroLogger = new DataLogger("Gyro", robot.telemetry, false);

		// Initialize the header rows
		opmodeLogger.addField("Category");
		opmodeLogger.addField("Message");
		opmodeLogger.firstLine();
		gyroLogger.addField("Global Angle");
		gyroLogger.firstLine();
	
		// Put data into the gyro log
		gyroLogger.addField(0);
		gyroLogger.newLine();

		// Put data into the opmode log
		opmodeLogger.addField("Count");
		opmodeLogger.addField("0");
		opmodeLogger.newLine();

	
		// Wait for the start button to be pressed.
		waitForStart();
		telemetry.addData("DataLoggerTest", "Robot Started");
		telemetry.update();

		// Loop until we are asked to stop, displaying the gyro info in telemetry
		int count = 0;
		while (opModeIsActive()) {
			gyro.getCurrentAngle();
			count++;
		}

		// Put data into the opmode log
		opmodeLogger.addField("Count");
		opmodeLogger.addField(count);
		opmodeLogger.newLine();
	
		// CLose the logs
		opmodeLogger.closeDataLogger();
		gyroLogger.closeDataLogger();
	}
}
