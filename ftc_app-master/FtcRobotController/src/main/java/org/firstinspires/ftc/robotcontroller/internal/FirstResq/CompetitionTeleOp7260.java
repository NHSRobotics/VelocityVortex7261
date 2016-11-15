package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.general.Movement;

/**
 * Created by 7260 on 1/15/2016.
 */

/**
 * TeleOp program that uses 2 controllers for the First ResQ Challenge 2015-2016.
 *
 * 1st controller has the actuator controls which include: Tape Measure Hook Angle and Extension Controls, Side Servo Arms
 *
 * 2nd controller has the movement controls which include: Joystick Tank Drive, Precision D-pad Drive, Inversion Drive
 */
public class CompetitionTeleOp7260 extends OpMode{

    DcMotor angle;                              //Motor that controls the angle of the tape-measure grappling hook
    DcMotor extension;                          //Motor that controls the extension of the tape-measure grappling hook
    Movement Chassis;                           //Class that holds the driving motors
    Servo leftFlipper;                          //Servo that rotates on the left-side of the chassis to hit the climber levers on the mountain
    Servo rightFlipper;                         //Servo that rotates on the right-side of the chassis to hit the climber levers on the mountain
    DcMotor chute;
    DcMotor scoop;
    Servo climber;

    Boolean inverse = false;                    //Holds whether the movement controls are inverted
    long last = System.currentTimeMillis();     //Used to check a time delay

    Boolean fastExtension = false;
    Boolean slowAngle = false;

    public void init(){
        //
        //Initializes and sets the hardware found through the config file
        //
        Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        angle = hardwareMap.dcMotor.get("angle");
        extension = hardwareMap.dcMotor.get("extension");
        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");
        chute = hardwareMap.dcMotor.get("chute");
        scoop = hardwareMap.dcMotor.get("scoop");
        climber = hardwareMap.servo.get("climber");

        climber.setPosition(1);
        leftFlipper.setPosition(0);   //Sets the left servo to the up position
        rightFlipper.setPosition(.58);    //Sets the right servo to the up position
    }

    public void loop(){

        //
        //Raises or lowers the angle of the grapple when the right joystick on the first controller
        //is pushed up or down, respectively
        //
        if (slowAngle) {
            if (gamepad1.right_stick_y < 0) {
                angle.setPower(-.075);
            } else if (gamepad1.right_stick_y > 0) {
                angle.setPower(.075);
            } else {
                angle.setPower(0);
            }
        } else {
            if (gamepad1.right_stick_y < 0) {
                angle.setPower(-.3);
            } else if (gamepad1.right_stick_y > 0) {
                angle.setPower(.15);
            } else {
                angle.setPower(0);
            }
        }

        //
        //Extends or retracts the tape-measure of the grapple when the left joystick on the first controller
        //is pushed up or down, respectively
        //

        if (fastExtension) {
            if (gamepad1.left_stick_y < 0) {
                extension.setPower(.5);
            } else if (gamepad1.left_stick_y > 0) {
                extension.setPower(-.5);
            } else {
                extension.setPower(0);
            }
        } else {
            if (gamepad1.left_stick_y < 0) {
                extension.setPower(1);
            } else if (gamepad1.left_stick_y > 0) {
                extension.setPower(-1);
            } else {
                extension.setPower(0);
            }
        }

        //
        //Controls the power of the chute
        //
        if(gamepad1.right_bumper){
            chute.setPower(-.75);
        } else if (gamepad1.left_bumper) {
            chute.setPower(.75);
        } else {
            chute.setPower(0);
        }

        //
        //Controls the scoop position
        //
        if (gamepad1.dpad_down){
            scoop.setPower(-1);
        } else if (gamepad1.dpad_up) {
            scoop.setPower(1);
        } else {
            scoop.setPower(0);
        }

        //
        //Makes the scoops go to a set location
        //

        //
        //Determines which control scheme to use based on the inversion variable
        //
        if (inverse) { //Inverted Controls

            //
            //Left joystick of controller 2 controls the left side motors
            //
            if (gamepad2.left_stick_y > 0) {
                Chassis.leftMotor.setPower(Math.pow(gamepad2.left_stick_y, 2));
            } else if (gamepad2.left_stick_y < 0) {
                Chassis.leftMotor.setPower(-1 * Math.pow(gamepad2.left_stick_y, 2));
            }

            //
            //Right joystick of controller 2 controls the right side motors
            //
            if (gamepad2.right_stick_y > 0) {
                Chassis.rightMotor.setPower(Math.pow(gamepad2.right_stick_y, 2));
            } else if (gamepad2.right_stick_y < 0) {
                Chassis.rightMotor.setPower(-1 * Math.pow(gamepad2.right_stick_y, 2));
            }

            //
            //The D-pad of controller 2 uses fine-tune controls to drive
            //
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

        } else { //Regular Controls
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

        //
        //Inverts the control scheme when the "a" button is pressed on the second controller
        //
        //A time check is added to allow for a delay, so the inversion is fully recognized
        //
        if(gamepad2.a && ( System.currentTimeMillis() - last) >= 1000) {
            if (inverse) {
                inverse = false;
            } else {
                inverse = true;
            }
            last = System.currentTimeMillis();
        }
        //
        //Rotates the left servo arm when the "a" button is pressed on the first controller
        //
        //A time check is added to allow for a delay, so the command is fully executed
        //
        if (gamepad2.x && (System.currentTimeMillis() - last >= 1000)) {
            if (leftFlipper.getPosition() == 0) {
                leftFlipper.setPosition(.58);
            } else {
                leftFlipper.setPosition(0);
            }
            last = System.currentTimeMillis();
        }

        //
        //Rotates the right servo arm when the "b" button is pressed on the first controller
        //
        //A time check is added to allow for a delay, so the command is fully executed
        //
        if (gamepad2.b && (System.currentTimeMillis() - last >= 1000)) {
            if (rightFlipper.getPosition() == 0) {
                rightFlipper.setPosition(.62);
            } else {
                rightFlipper.setPosition(0);
            }
            last = System.currentTimeMillis();
        }

        if (gamepad2.right_bumper && (System.currentTimeMillis() - last >= 1000)) {
            if (leftFlipper.getPosition() < .55 && leftFlipper.getPosition() > .6) {
                leftFlipper.setPosition(.58);
            } else {
                leftFlipper.setPosition(0);
            }
            last = System.currentTimeMillis();
        }

        if (gamepad2.right_trigger > 0 && (System.currentTimeMillis() - last >= 1000)) {
            if (leftFlipper.getPosition() < .6) {
                leftFlipper.setPosition(.65);
            } else {
                leftFlipper.setPosition(0);
            }
            last = System.currentTimeMillis();
        }

        //
        //Rotates the right servo arm when the "b" button is pressed on the first controller
        //
        //A time check is added to allow for a delay, so the command is fully executed
        //
        if (gamepad2.left_bumper && (System.currentTimeMillis() - last >= 1000)) {
            if (rightFlipper.getPosition() > .6 && rightFlipper.getPosition() < .7) {
                rightFlipper.setPosition(0);
            } else {
                rightFlipper.setPosition(.62);
            }
            last = System.currentTimeMillis();
        }

        if (gamepad2.left_trigger > 0 && (System.currentTimeMillis() - last >= 1000)) {
            if (rightFlipper.getPosition() < .75) {
                rightFlipper.setPosition(.8);
            } else {
                rightFlipper.setPosition(0);
            }
            last = System.currentTimeMillis();
        }


        //
        //Switching to fastExtension
        //
        if (gamepad1.x && (System.currentTimeMillis() - last >= 1000)) {
            if (fastExtension) {
                fastExtension = false;
            } else {
                fastExtension = true;
            }
            last = System.currentTimeMillis();
        }

        //
        //Switching to slowAngle
        //
        if (gamepad1.y && (System.currentTimeMillis() - last >= 1000)) {
            if (slowAngle) {
                slowAngle = false;
            } else {
                slowAngle = true;
            }
            last = System.currentTimeMillis();
        }

        telemetry.addData("leftFlip:", leftFlipper.getPosition());
    }
}

