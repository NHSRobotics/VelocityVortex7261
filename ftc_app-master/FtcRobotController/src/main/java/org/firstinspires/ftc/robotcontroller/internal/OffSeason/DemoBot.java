package org.firstinspires.ftc.robotcontroller.internal.OffSeason;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Robotics on 6/8/2016.
 */
public class DemoBot extends OpMode {
    DcMotor rightWheel;
    DcMotor leftWheel;
    //DcMotor flipper;
    //DcMotor lift;

    //Servo scoop;

    @Override
    public void init(){
        rightWheel = hardwareMap.dcMotor.get("right");
        leftWheel = hardwareMap.dcMotor.get("left");

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        if (gamepad1.left_stick_y > 0){
            leftWheel.setPower(Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.left_stick_y < 0){
            leftWheel.setPower(-1 * Math.pow(gamepad1.left_stick_y, 2));
        } else {
            leftWheel.setPower(0);
        }

        if (gamepad1.right_stick_y > 0){
            rightWheel.setPower(Math.pow(gamepad1.right_stick_y, 2));
        } else if (gamepad1.right_stick_y < 0){
            rightWheel.setPower(-1 * Math.pow(gamepad1.right_stick_y, 2));
        } else {
            rightWheel.setPower(0);
        }
    }
}
