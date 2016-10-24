package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 4/13/2016.
 */
public class TestMovement extends OpMode {

    DcMotor RightMotorOne;
    DcMotor RightMotorTwo;
    DcMotor LeftMotorOne;
    DcMotor LeftMotorTwo;
    boolean ToggleServo = false;
    Servo ServoArm;

    long last = System.currentTimeMillis();

    public void init (){

        RightMotorOne = hardwareMap.dcMotor.get("RightMotorOne");
        RightMotorTwo = hardwareMap.dcMotor.get("RightMotorTwo");
        LeftMotorOne = hardwareMap.dcMotor.get("LeftMotorOne");
        LeftMotorTwo = hardwareMap.dcMotor.get("LeftMotorTwo");
        ServoArm = hardwareMap.servo.get("ServoArm");

        ServoArm.setPosition(.1);
    }

    @Override
    public void loop() {

        if(gamepad1.left_stick_y > 0){

            LeftMotorOne.setPower(-1);
            LeftMotorTwo.setPower(-1);

        }else if (gamepad1.left_stick_y < 0 ){

            LeftMotorOne.setPower(1);
            LeftMotorTwo.setPower(1);

        }else {

            LeftMotorOne.setPower(0);
            LeftMotorTwo.setPower(0);

        }

        if(gamepad1.right_stick_y > 0){

            RightMotorOne.setPower(1);
            RightMotorTwo.setPower(1);

        }else if (gamepad1.right_stick_y < 0){

            RightMotorOne.setPower(-1);
            RightMotorTwo.setPower(-1);

        }else {

            RightMotorOne.setPower(0);
            RightMotorTwo.setPower(0);

        }

        if(gamepad1.a && (System.currentTimeMillis() - last ) >= 1000) {
            if (ToggleServo == false) {
                ServoArm.setPosition(.9);
                ToggleServo = true;
            } else if (ToggleServo == true) {
                ServoArm.setPosition(.1);
                ToggleServo = false;
            }
            last = System.currentTimeMillis();
        }

    }
}
