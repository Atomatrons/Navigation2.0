package org.firstinspires.ftc.teamcode.navigation;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.ShivaRobot;

/**
 * This class detects and reports which alliance the robot is on.
 * The alliance is detected by reading the color of the alliance marker, using a color sensor.
 */
public class ShivaAlliance {
	// The detected alliance. Only vaid after calling detectCurrent
	public Alliance current;

	// You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
	// normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
	// can give very low values (depending on the lighting conditions), which only use a small part
	// of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
	// you should use a smaller gain than in dark conditions. If your gain is too high, all of the
	// colors will report at or near 1, and you won't be able to determine what color you are
	// actually looking at. For this reason, it's better to err on the side of a lower gain
	// (but always greater than or equal to 1).
	public static final float gain = 2.0f;

    // The first element (0) will contain the hue,
	// the second element (1) will contain the saturation,
	// and the third element (2) will contain the value.
	// See: http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];


	public Alliance detectCurrent(ShivaRobot robot){
		// If possible, turn the light on in the beginning (it might already be on anyway,
		// we just make sure it is if we can).
		// NOTE: My understanding is the RevRobotics light sensor is NOT controllable from software.
		//		 If it is, and this code actuall yturns the light on, we should add a delay
		//		 between ehre and reading the color, to allow time for the light to turn on.
		if (robot.allianceColorSensor instanceof SwitchableLight) {
			((SwitchableLight)robot.allianceColorSensor).enableLight(true);
		}

		// Tell the sensor our desired gain value
		robot.allianceColorSensor.setGain(gain);

		// Get the normalized colors from the sensor
		NormalizedRGBA colors = robot.allianceColorSensor.getNormalizedColors();

		// Turn the light off.
		if (robot.allianceColorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)robot.allianceColorSensor;
            light.enableLight(!light.isLightOn());
		}

		// Convert the RGB colors to HSV
		Color.colorToHSV(colors.toColor(), hsvValues);

		this.displayInfo(robot, colors);

		return Alliance.RED;
	}

	// Display info about the color sensor to the Driver Station
	private void displayInfo(ShivaRobot robot, NormalizedRGBA colors) {
		robot.telemetry.addLine()
				.addData("Red", "%.3f", colors.red)
				.addData("Green", "%.3f", colors.green)
				.addData("Blue", "%.3f", colors.blue);
		robot.telemetry.addLine()
				.addData("Hue", "%.3f", hsvValues[0])
				.addData("Saturation", "%.3f", hsvValues[1])
				.addData("Value", "%.3f", hsvValues[2]);
		robot.telemetry.addData("Alpha", "%.3f", colors.alpha);

		/* If this color sensor also has a distance sensor, display the measured distance.
		* Note that the reported distance is only useful at very close range, and is impacted by
		* ambient light and surface reflectivity. */
		if (robot.allianceColorSensor instanceof DistanceSensor) {
			robot.telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) robot.allianceColorSensor).getDistance(DistanceUnit.CM));
		}

		robot.telemetry.update();
	}
}
