package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 7260 on 2/22/2016.
 */
public class TestingMovement extends OpMode {

    DcMotor RightDcMotor;
    DcMotor LeftDcMotor;

    DcMotor ArmMotor;

    Servo ArmServo;

    boolean ToggleServo = true;

    long last = System.currentTimeMillis();

    public void init(){
        RightDcMotor = hardwareMap.dcMotor.get("RightMotor");
        LeftDcMotor = hardwareMap.dcMotor.get("LeftMotor");
        ArmMotor = hardwareMap.dcMotor.get("ArmMotor");

        ArmServo = hardwareMap.servo.get("ArmServo");
        ArmServo.setPosition(0);

    }

    public void loop(){

        //
        //Tank Drive Section
        //
        if (gamepad1.left_stick_y>0){

            LeftDcMotor.setPower(0.5);

        }else if (gamepad1.left_stick_y<0){

            LeftDcMotor.setPower(-0.5);

        }else if (gamepad1.left_stick_y==0){

            LeftDcMotor.setPower(0.0);
        }

        if (gamepad1.right_stick_y>0){

            RightDcMotor.setPower(-0.5);

        }else if (gamepad1.right_stick_y<0){

            RightDcMotor.setPower(0.5);

        }else if (gamepad1.right_stick_y==0){

            RightDcMotor.setPower(0.0);

        }

        if (gamepad2.left_stick_y > 0){

            ArmMotor.setPower(.3);

        }else if (gamepad2.left_stick_y < 0){

            ArmMotor.setPower(-.3);

        }else {

            ArmMotor.setPower(0);

        }

        if (gamepad2.right_stick_y > 0) {

            ArmServo.setPosition(1.0);

        }else if (gamepad2.right_stick_y < 0){

            ArmServo.setPosition(0);

        }else {

            ArmServo.setPosition(0.5);

        }

        telemetry.addData("ToggleServo: ", ToggleServo);
    }

}
