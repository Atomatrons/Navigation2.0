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
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.navigation.DriveTrain;
import org.firstinspires.ftc.teamcode.navigation.Gyro;

/**
 * This op-mode tests the Alliance classes
 */
@Autonomous(name = "Reset Slides", group = "Test")
public class ResetSlides extends LinearOpMode {

  ShivaRobot robot = new ShivaRobot();

  public void runOpMode() throws InterruptedException{
    robot.init(telemetry, hardwareMap);

    robot.slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.slides_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

}
}
