package com.qualcomm.ftcrobotcontroller.opmodes.OffSeason;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 5/6/2016.
 */
public class FreshmenBot1 extends OpMode {

    DcMotor rightWheel;
    DcMotor leftWheel;
    //DcMotor flipper;
    DcMotor lift;

    Servo scoop;

    @Override
    public void init(){
        rightWheel = hardwareMap.dcMotor.get("right");
        leftWheel = hardwareMap.dcMotor.get("left");

        //flipper = hardwareMap.dcMotor.get("flipper");
        lift = hardwareMap.dcMotor.get("lift");

        scoop= hardwareMap.servo.get("scoop");
        scoop.setPosition(.5);

        rightWheel.setDirection(DcMotor.Direction.REVERSE);
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

        if (gamepad2.left_stick_y > 0) {
            lift.setPower(1);
        } else if (gamepad2.left_stick_y < 0) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        if (gamepad2.right_stick_y < 0 && scoop.getPosition() < .99){
            scoop.setPosition(scoop.getPosition() + .005);
        }

        if (gamepad2.right_stick_y > 0 && scoop.getPosition() > .01){
            scoop.setPosition(scoop.getPosition() - .005);
        }

        /*if (gamepad2.dpad_left){
            flipper.setPower(.3);
        } else if (gamepad2.dpad_right) {
            flipper.setPower(-.3);
        } else {
            flipper.setPower(0);
        }*/
    }
}
