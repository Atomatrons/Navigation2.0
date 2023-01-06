package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.ShivaRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class provides a high-level interface to the Distance Sensors.
 */
public class Distance {
    private Telemetry telemetry = null;
    private DistanceSensor distance_left = null;
    private DistanceSensor distance_right = null;

    public static final double DISTANCE_MIN = 10.0;    // values smaller than this seem to be inaccurate
    public static final double DISTANCE_MAX = 50.0;  // values larger than this seem to be inaccurate and/or not useful

    public void init(ShivaRobot robot)
    {
        this.telemetry = robot.telemetry;
        this.distance_left = robot.distance_left;
        this.distance_right = robot.distance_right;
    }

    public boolean targetLocked() 
    {
        double range = distance_left.getDistance(DistanceUnit.CM);
        if (range > DISTANCE_MIN && range < DISTANCE_MAX) 
        {
            return true;
        }

        range = distance_right.getDistance(DistanceUnit.CM);
        if (range > DISTANCE_MIN && range < DISTANCE_MAX) 
        {
            return true;
        }

        return false;
    }

    public double distance() 
    {
        double range = distance_left.getDistance(DistanceUnit.CM);
        if (range > DISTANCE_MIN && range < DISTANCE_MAX) 
        {
            return range;
        }

        range = distance_right.getDistance(DistanceUnit.CM);
        if (range > DISTANCE_MIN && range < DISTANCE_MAX) 
        {
            return range;
        }

        return 0.0;
    }

    public double distance(boolean verbose) 
    {
        double result = 0;

        double range = distance_left.getDistance(DistanceUnit.CM);
        if (verbose) {
            this.telemetry.addData("Distance Left", range);
        }
        if (range > DISTANCE_MIN && range < DISTANCE_MAX) 
        {
            if (verbose) {
                this.telemetry.addData("Distance Sensor", "LEFT");
            }
            result = range;
        }

        range = distance_right.getDistance(DistanceUnit.CM);
        if (verbose) {
            this.telemetry.addData("Distance RIght", range);
        }
        if (range > DISTANCE_MIN && range < DISTANCE_MAX) 
        {
            if (verbose) {
                this.telemetry.addData("Distance Sensor", "RIGHT");
            }
            result = range;
        }

        return result;
    }
}
