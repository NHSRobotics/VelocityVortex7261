package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcontroller.internal.general.Movement;

import java.util.Date;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by 7260 on 10/19/2016.
 */
public class FirstCompTeleOp extends OpMode{

    Movement chassis;

    DcMotor first;
    DcMotor second;
    DcMotor leftshoot;
    DcMotor rightshoot;
    DcMotor ldraw;
    DcMotor rdraw;

    Servo paddle;
    boolean inverted = false;

    long padTime = System.currentTimeMillis();
    long invertTime = System.currentTimeMillis();

    public void init() {
        chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));

        first = hardwareMap.dcMotor.get("first");
        second = hardwareMap.dcMotor.get("second");
        leftshoot = hardwareMap.dcMotor.get("leftshoot");
        rightshoot = hardwareMap.dcMotor.get("rightshoot");
        ldraw = hardwareMap.dcMotor.get("ldraw");
        rdraw = hardwareMap.dcMotor.get("rdraw");

        paddle = hardwareMap.servo.get("paddle");

        leftshoot.setDirection(DcMotor.Direction.REVERSE);
        ldraw.setDirection(DcMotor.Direction.REVERSE);
        paddle.setPosition(0);

        //setCameraDownsampling(8);
        //super.init();
    }

    public void loop() {
        /*if (!inverted){
            //Regular tank drive
            if(gamepad1.left_stick_y > 0) {
                chassis.leftMotor.setPower(1 * Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.left_stick_y < 0) {
                chassis.leftMotor.setPower(-1 * Math.pow(gamepad1.left_stick_y, 2));
            } else {
                chassis.leftMotor.setPower(0);
            }
            if(gamepad1.right_stick_y > 0) {
                chassis.rightMotor.setPower(1 * Math.pow(gamepad1.right_stick_y, 2));
            } else if (gamepad1.right_stick_y < 0) {
                chassis.rightMotor.setPower(-1 * Math.pow(gamepad1.right_stick_y, 2));
            } else {
                chassis.rightMotor.setPower(0);
            }
            //Regular set drive
            if(gamepad1.dpad_up){
                chassis.forwardDrive(1);
            } else if (gamepad1.dpad_down) {
                chassis.reverseDrive(1);
            } else if (gamepad1.dpad_right) {
                chassis.rightSpin(1);
            } else if (gamepad1.dpad_left) {
                chassis.leftSpin(1);
            } else {
                chassis.stopDrive();
            }
        } else {
            //Inverted tank drive
            if(gamepad1.left_stick_y > 0) {
                chassis.leftMotor.setPower(-1 * Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.left_stick_y < 0) {
                chassis.leftMotor.setPower(1 * Math.pow(gamepad1.left_stick_y, 2));
            } else {
                chassis.leftMotor.setPower(0);
            }
            if(gamepad1.right_stick_y > 0) {
                chassis.rightMotor.setPower(-1 * Math.pow(gamepad1.right_stick_y, 2));
            } else if (gamepad1.right_stick_y < 0) {
                chassis.rightMotor.setPower(1 * Math.pow(gamepad1.right_stick_y, 2));
            } else {
                chassis.rightMotor.setPower(0);
            }
            //Inverted set drive
            if(gamepad1.dpad_up){
                chassis.reverseDrive(1);
            } else if (gamepad1.dpad_down) {
                chassis.forwardDrive(1);
            } else if (gamepad1.dpad_right) {
                chassis.leftSpin(1);
            } else if (gamepad1.dpad_left) {
                chassis.rightSpin(1);
            } else {
                chassis.stopDrive();
            }
        }*/

        if (!inverted){
            if (gamepad1.left_stick_y > 0) {
                chassis.forwardDrive(Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.left_stick_y < 0) {
                chassis.reverseDrive(Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.right_stick_x > 0) {
                chassis.rightSpin(1);
            } else if (gamepad1.right_stick_x < 0) {
                chassis.leftSpin(1);
            } else {
                chassis.stopDrive();
            }
        } else {
            if (gamepad1.left_stick_y > 0) {
                chassis.reverseDrive(Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.left_stick_y < 0) {
                chassis.forwardDrive(Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.right_stick_x > 0) {
                chassis.leftSpin(1);
            } else if (gamepad1.right_stick_x < 0) {
                chassis.rightSpin(1);
            } else {
                chassis.stopDrive();
            }
        }

        //Conveyor belt
        if (gamepad2.dpad_up) {
            second.setPower(-.5);
        } else if (gamepad2.dpad_down){
            second.setPower(.5);
        } else {
            second.setPower(0);
        }

        //Shooter
        if (gamepad2.x){
            leftshoot.setPower(-1);
            rightshoot.setPower(-1);
        } else {
            leftshoot.setPower(0);
            rightshoot.setPower(0);
        }

        //Collector
        if (gamepad2.b){
            first.setPower(1);
        } else {
            first.setPower(0);
        }

        //Paddle
        if (gamepad2.a && System.currentTimeMillis() - padTime > 1000){
            if (paddle.getPosition() < .05){
                paddle.setPosition(1);
            } else {
                paddle.setPosition(0);
            }
            padTime = System.currentTimeMillis();
        }

        //Inverse Drive
        if (gamepad1.a && System.currentTimeMillis() - invertTime > 1000) {
            if (inverted){
                inverted = false;
            } else {
                inverted = true;
            }
        }

        //Drawbridge
        if (gamepad2.right_bumper) {
            ldraw.setPower(.3);
            rdraw.setPower(.3);
        } else if (gamepad2.left_bumper) {
            ldraw.setPower(-.3);
            rdraw.setPower(-.3);
        } else {
            ldraw.setPower(0);
            rdraw.setPower(0);
        }
    }

}
