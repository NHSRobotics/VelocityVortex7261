package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by phama21 on 9/12/2015.
 */

/*
    MovementTeleOp class creates the common methods, variables, and objects for use the
    basic movement in other TeleOp files.
 */

public class MovementTeleOp {

    private DcMotor leftmotor;          //Declares DcMotor object
    private DcMotor rightmotor;         //Declares DcMotor object
    final double THRESHOLD = 0.1;       //Value that the gamepad joystick has to surpass before the motors will run

    /*
        Constructor method that sets the DcMotor objects and reverses the direction of the right motor
        @param left     the left side motors
        @param right    the right side motors
     */
    /*public MovementTeleOp(DcMotor left, DcMotor right) {
        leftmotor = left;
        rightmotor = right;

        right.setDirection(DcMotor.Direction.REVERSE);
    }*/

    /*
        Overloaded tankDrive method that takes the gamepad and sends the data value of the left and right joystick
        to the other tankDrive method
        @param  controller  the gamepad that values are taken from
     */
    public void tankDrive(Gamepad controller) {
        tankDrive(controller.left_stick_y, controller.right_stick_y);
    }

    /*
        Overloaded tankDrive method that send the values of the gamepad and also whether the inputs are squared or not
        @param  controller  the gamepad that values are taken from
        @param  squared     boolean value that states whether the input is squared
    */
    public void tankDrive(Gamepad controller, boolean squared) {
        tankDrive(controller.left_stick_y, controller.right_stick_y, squared);
    }

    /*
        tankDrive method that takes the gamepad joystick values and sets it to the power of the motors
        @param  leftValue   double value of the left joystick value
        @param  rightValue  double value of the right joystick value
     */
    public void tankDrive(double leftValue, double rightValue) {
        if (Math.abs(leftValue) > THRESHOLD) {  //To prevent idle drive, the absolute value of the joystick value as to be over a threshold
            leftmotor.setPower(leftValue);
        }
        if (Math.abs(rightValue) > THRESHOLD) {
            rightmotor.setPower(rightValue);
        }
    }

    /*
        tankDrive method that squares the input, instead of direct output, so the the output is exponential
        and the lower values are slower
        @param  leftValue       the left joystick value
        @param  rightValue      the right joystick value
        @param  squareInputs    boolean value to determine whether the inputs will be squared
     */
    public void tankDrive(double leftValue, double rightValue, boolean squareInputs) {

        if (squareInputs) {
            if (leftValue >= 0.0) {
                leftValue = (leftValue * leftValue);
            } else if (leftValue <= 0.0) {
                leftValue = -(leftValue * leftValue);
            }
            if (rightValue >= 0.0) {
                rightValue = (rightValue * rightValue);
            } else if (rightValue <= 0.0){
                rightValue = -(rightValue * rightValue);
            }
        }

        tankDrive(leftValue, rightValue);
    }


    final private long SECONDS = 1000;  //Constant of how long the movement commands will run
    final private double POWER = 0.5;   //Constant of how much power the movement commands will run on

    /*
        nudgeDrive method that takes the button input from the gamepad to do control movement outputs
        @param button   the int that determines which button was pressed
     */
    /*public void nudgeDrive(int button){

        switch (button) {
            //Drives forward when button 1 is pressed
            case 1:
                forwardDrive(leftmotor, rightmotor, POWER);

                break;
            //Turns left when button 2 is pressed
            case 2:
                leftSpin(leftmotor, rightmotor, POWER);

                break;
            //Reveres when button 3 is pressed
            case 3:
                reverseDrive(leftmotor, rightmotor, POWER);

                break;
            //Turns right when button 4 is pressed
            case 4:
                rightSpin(leftmotor,rightmotor, POWER);

                break;
        }

    }*/

}
