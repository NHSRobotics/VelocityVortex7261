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
public class CompetitionTeleOp7261 extends OpMode{

    DcMotor angle;                              //Motor that controls the angle of the tape-measure grappling hook
    DcMotor extension;                          //Motor that controls the extension of the tape-measure grappling hook
    Movement Chassis;                           //Class that holds the driving motors
    Servo leftFlipper;                          //Servo that rotates on the left-side of the chassis to hit the climber levers on the mountain
    Servo rightFlipper;                         //Servo that rotates on the right-side of the chassis to hit the climber levers on the mountain
    Servo rightsideButton;                      //Servo that controls the right arm that hits the beacon during autonomous
    Servo leftsideButton;                       //Servo that controls the left arm that hits the beacon during autonomous
    Servo climberArm;                           //Servo that controls the arm that dumps the climbers during autonomous
    Servo colorArm;                             //Servo that extends out the color sensor during autonomous
    Servo mantisLeft;                           //Servo that controls the arm that raises the all-clear signal
    Servo mantisRight;                          //Servo that controls the arm that raises the all-clear signal

    Boolean inverse = false;                    //Holds whether the movement controls are inverted
    long last = System.currentTimeMillis();     //Used to check a time delay

    Boolean fastExtension = false;              //Holds whether the tape measure extends at a faster speed
    Boolean slowAngle = false;                  //Holds whether the tape measure changes angle at a slower speed

    Boolean ACout = false;                      //Holds whether the all-clear signal arms are out
    Boolean leftAC = false;

    public void init() {
        //
        //Initializes and sets the hardware found through the config file
        //
        Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        angle = hardwareMap.dcMotor.get("angle");
        extension = hardwareMap.dcMotor.get("extension");
        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");
        rightsideButton = hardwareMap.servo.get("rightButton");
        leftsideButton = hardwareMap.servo.get("leftButton");
        climberArm = hardwareMap.servo.get("climber");
        colorArm = hardwareMap.servo.get("colorArm");
        mantisLeft = hardwareMap.servo.get("mantisL");
        mantisRight = hardwareMap.servo.get("mantisR");


        leftFlipper.setPosition(0);         //Sets the left servo to the up position
        rightFlipper.setPosition(1);        //Sets the right servo to the up position
        climberArm.setPosition(.95);        //Sets the climber arm to the starting position
        colorArm.setPosition(.6);           //Sets the color sensor arm to the starting position
        rightsideButton.setPosition(1);     //Sets the button to the down position
        leftsideButton.setPosition(0);      //Sets the button to the down position
        mantisLeft.setPosition(1);          //Sets the left mantis arm to the backward position
        mantisRight.setPosition(0);         //Sets the right mantis arm to the backward position
    }


    public void loop(){

        //
        //Raises or lowers the angle of the grapple when the right joystick on the first controller
        //is pushed up or down, respectively.
        //
        //The choice of speed is dependent on boolean slowAngle
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
                angle.setPower(-.25);
            } else if (gamepad1.right_stick_y > 0) {
                angle.setPower(.45);
            } else {
                angle.setPower(0);
            }
        }

        //
        //Extends or retracts the tape-measure of the grapple when the left joystick on the first controller
        //is pushed up or down, respectively
        //

        //if (fastExtension) {
            if (gamepad1.left_stick_y < 0) {
                extension.setPower(1);
            } else if (gamepad1.left_stick_y > 0) {
                extension.setPower(-1);
            } else {
                extension.setPower(0);
            }
        //} else {
        /*
            if (gamepad1.left_stick_y < 0) {
                extension.setPower(.5);
            } else if (gamepad1.left_stick_y > 0) {
                extension.setPower(-.5);
            } else {
                extension.setPower(0);
            }
        }
       */

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
                Chassis.forwardDrive(.75);
            } else if (gamepad2.dpad_up){
                Chassis.reverseDrive(.75);
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
                Chassis.reverseDrive(.75);
            } else if (gamepad2.dpad_up){
                Chassis.forwardDrive(.75);
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

        /*
        if (gamepad2.left_trigger == 1 && (System.currentTimeMillis() - last >= 1000)) {
            if (leftFlipper.getPosition() < .05) {
                leftFlipper.setPosition(.75);
            } else {
                leftFlipper.setPosition(0);
            }
            last = System.currentTimeMillis();
        }
        */

        if (gamepad2.left_trigger > 0 && (System.currentTimeMillis() - last >= 1000)) {
            if (leftFlipper.getPosition() < .05) {
                leftFlipper.setPosition(.75);
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
        if (gamepad2.right_trigger == 1 && (System.currentTimeMillis() - last >= 1000)) {
            if (rightFlipper.getPosition() > .95) {
                rightFlipper.setPosition(.25);
            } else {
                rightFlipper.setPosition(1);
            }
            last = System.currentTimeMillis();
        }
/*
        if (gamepad1.left_trigger > 0 && (System.currentTimeMillis() - last >= 1000)) {
            if (rightFlipper.getPosition() != .8) {
                rightFlipper.setPosition(.8);
            } else {
                rightFlipper.setPosition(.1);
            }
            last = System.currentTimeMillis();
        }
*/
        if(gamepad1.dpad_right){
            if (climberArm.getPosition() < .95) {
                climberArm.setPosition(climberArm.getPosition() + .005);
            }
        }

        if (gamepad1.dpad_left){
            if (climberArm.getPosition() > .05) {
                climberArm.setPosition(climberArm.getPosition() - .005);
            }
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

        if (gamepad1.right_trigger == 1 && (System.currentTimeMillis() - last >= 1000)) {
            if (ACout) {
                mantisRight.setPosition(0);
                mantisLeft.setPosition(1);
                ACout = false;
            } else {
                mantisRight.setPosition(1);
                mantisLeft.setPosition(0);
                ACout = true;
            }
            last = System.currentTimeMillis();
        }

      /*  if (gamepad1.left_trigger == 1 && (System.currentTimeMillis() - last >= 1000)) {
            if (leftAC) {
                leftAC = false;
            } else {
                leftAC = true;
            }
            last = System.currentTimeMillis();
        }
*/
        telemetry.addData("Encoder Count", Chassis.leftMotor.getCurrentPosition());
        telemetry.addData("Left Flipper Position", leftFlipper.getPosition());
        telemetry.addData("Right Flipper Position", rightFlipper.getPosition());
        telemetry.addData("Left Mantis Position", mantisLeft.getPosition());
        telemetry.addData("Right Mantis Position", mantisRight.getPosition());
    }
}
