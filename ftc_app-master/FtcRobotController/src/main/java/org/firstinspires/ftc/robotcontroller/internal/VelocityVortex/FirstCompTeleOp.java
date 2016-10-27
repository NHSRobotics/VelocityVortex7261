package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FirstResq.Movement;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by 7260 on 10/19/2016.
 */
public class FirstCompTeleOp extends OpModeCamera{

    Movement chassis;
    int ds2 = 2;

    public void init() {
        chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        setCameraDownsampling(8);
        super.init();
    }

    public void loop() {
        if (gamepad1.left_stick_y > 0){
            chassis.forwardDrive(Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.left_stick_y < 0){
            chassis.reverseDrive(Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.right_stick_x > 0) {
            chassis.rightSpin(.3*Math.pow(gamepad1.right_stick_x, 2));
        } else if (gamepad1.right_stick_x < 0){
            chassis.leftSpin(.3*Math.pow(gamepad1.right_stick_x, 2));
        } else {
            chassis.stopDrive();
        }

        if(imageReady()){
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
            telemetry.addData("Width: ", rgbImage.getWidth());
            telemetry.addData("Height: ", rgbImage.getHeight());
        }
    }
}
