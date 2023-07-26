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
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.navigation.DriveTrain;
import org.firstinspires.ftc.teamcode.navigation.Gyro;

@Autonomous(name = "Push Cone", group = "Test")
public class PushCone extends LinearOpMode {
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    
    

    @Override
    public void runOpMode() {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        
    

        //For pushing the cone when the robot is left of the blue (or red) triangle
        waitForStart();

        telemetry.addData("Moving", "Strafe Left");
      telemetry.update();
      // Strafes Left
      front_left.setPower(0.3);
      front_right.setPower(0.3);
      back_left.setPower(-0.3);
      back_right.setPower(-0.3);
      // Moves forwards
      sleep(1100);
      telemetry.addData("Moving", "Forward");
      telemetry.update();
      front_left.setPower(-0.3);
      front_right.setPower(0.3);
      back_left.setPower(-0.3);
      back_right.setPower(0.3);
      // Moves backwards
      sleep(900);
      telemetry.addData("Moving", "Backward");
      telemetry.update();
      front_left.setPower(0.3);
      front_right.setPower(-0.3);
      back_left.setPower(0.3);
      back_right.setPower(-0.3);
      // Strafes Right
      sleep(900);
      telemetry.addData("Moving", "Strafe Right");
      telemetry.update();
      front_left.setPower(-0.3);
      front_right.setPower(-0.3);
      back_left.setPower(0.3);
      back_right.setPower(0.3);
      // Goes forward
      sleep(1100);
      telemetry.addData("Moving", "Forward");
      telemetry.update();
      front_left.setPower(-0.5);
      front_right.setPower(0.5);
      back_left.setPower(-0.5);
      back_right.setPower(0.5);
      // Stops
      sleep(1000);
      telemetry.addData("Status", "Stopping");
      telemetry.update();
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      sleep(1000);
    
    
    
    
    
    
    
    }
}
