package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.general.Movement;

/**
 * Created by 7260 on 1/15/2016.
 */
public class testGrappling extends OpMode{
    DcMotor angle;
    DcMotor extension;
    Movement Chassis;
    Servo leftsideArm;
    Servo rightsideArm;

    Boolean inverse = false;
    long last = System.currentTimeMillis();

    public void init(){
        Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        angle = hardwareMap.dcMotor.get("angle");
        extension = hardwareMap.dcMotor.get("extension");
        leftsideArm = hardwareMap.servo.get("leftsideArm");
        rightsideArm = hardwareMap.servo.get("rightsideArm");

        leftsideArm.setPosition(.58);
        rightsideArm.setPosition(0);
    }

    public void loop(){
        if(gamepad1.right_stick_y < 0){
            angle.setPower(-.15);
        } else if (gamepad1.right_stick_y > 0) {
            angle.setPower(.15);
        } else {
            angle.setPower(0);
        }

        if (gamepad1.left_stick_y < 0) {
            extension.setPower(.5);
        } else if (gamepad1.left_stick_y > 0) {
            extension.setPower(-.5);
        } else {
            extension.setPower(0);
        }

        if (inverse) {
            if (gamepad2.left_stick_y > 0) {
                Chassis.leftMotor.setPower(Math.pow(gamepad2.left_stick_y, 2));
            } else if (gamepad2.left_stick_y < 0) {
                Chassis.leftMotor.setPower(-1 * Math.pow(gamepad2.left_stick_y, 2));
            }

            if (gamepad2.right_stick_y > 0) {
                Chassis.rightMotor.setPower(Math.pow(gamepad2.right_stick_y, 2));
            } else if (gamepad2.right_stick_y < 0) {
                Chassis.rightMotor.setPower(-1 * Math.pow(gamepad2.right_stick_y, 2));
            }

            if (gamepad2.dpad_down){
                Chassis.forwardDrive(.3);
            } else if (gamepad2.dpad_up){
                Chassis.reverseDrive(.3);
            } else if (gamepad2.dpad_right){
                Chassis.rightSpin(.5);
            } else if (gamepad2.dpad_left){
                Chassis.leftSpin(.5);
            }

            if (gamepad2.left_stick_y == 0 && gamepad2.right_stick_y == 0 && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.dpad_down){
                Chassis.stopDrive();
            }
        } else {
            if (gamepad2.left_stick_y > 0) {
                Chassis.rightMotor.setPower(-1 * Math.pow(gamepad2.left_stick_y, 2));
            } else if (gamepad2.left_stick_y < 0) {
                Chassis.rightMotor.setPower(Math.pow(gamepad2.left_stick_y, 2));
            }

            if (gamepad2.right_stick_y > 0) {
                Chassis.leftMotor.setPower(-1 * Math.pow(gamepad2.right_stick_y, 2));
            } else if (gamepad2.right_stick_y < 0) {
                Chassis.leftMotor.setPower(Math.pow(gamepad2.right_stick_y, 2));
            }

            if (gamepad2.dpad_down){
                Chassis.reverseDrive(.3);
            } else if (gamepad2.dpad_up){
                Chassis.forwardDrive(.3);
            } else if (gamepad2.dpad_right){
                Chassis.rightSpin(.5);
            } else if (gamepad2.dpad_left){
                Chassis.leftSpin(.5);
            }

            if (gamepad2.left_stick_y == 0 && gamepad2.right_stick_y == 0 && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.dpad_down){
                Chassis.stopDrive();
            }
        }

        if(gamepad2.a && ( System.currentTimeMillis() - last) >= 1000) {
            if (inverse) {
                inverse = false;
            } else {
                inverse = true;
            }
            last = System.currentTimeMillis();
        }

        if (gamepad1.a && (System.currentTimeMillis() - last >= 1000)) {
            if (leftsideArm.getPosition() == 0) {
                leftsideArm.setPosition(.58);
            } else {
                leftsideArm.setPosition(0);
            }
            last = System.currentTimeMillis();
        }

        if (gamepad1.b && (System.currentTimeMillis() - last >= 1000)) {
            if (rightsideArm.getPosition() == 0) {
                rightsideArm.setPosition(.62);
            } else {
                rightsideArm.setPosition(0);
            }
            last = System.currentTimeMillis();
        }
        //telemetry.addData("last:", last);
        //telemetry.addData("now:", System.currentTimeMillis());
    }
}
