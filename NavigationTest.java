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

import org.firstinspires.ftc.teamcode.navigation;

/**
 * This op-mode tests the Gyro class
 */
@Autonomous(name = "Navigation Test", group = "Test")
public class NavigationTest extends LinearOpMode {

	ShivaRobot robot = new ShivaRobot();
	Gyro gyro = new Gyro();
    Navigation nav = new Navigation();
    GPS gps = new GPS();
    DriveTrain driveTrain = new DriveTrain(); 

    Pose endPose = new Pose(2, 3, 90); 

	/**
	 * Test Navigation Code
	 */
	@Override
	public void runOpMode() {
		// Initialize the robot interface
		robot.init(telemetry, hardwareMap);
		gyro.init(robot);
        gps.init(robot, gyro);
        driveTrain.init(robot, gyro);
        nav.init(GPS.getCurrentPose()); 

        
		// Wait for the start button to be pressed.
		waitForStart();

        nav.goToPose(endPose, new PathXY());

		// Loop until we are asked to stop, displaying the gyro info in telemetry
		while (opModeIsActive()) {

        }
	}
}
