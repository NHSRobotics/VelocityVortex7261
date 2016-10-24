package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 7260 on 2/22/2016.
 */
public class NonTankTestingMovement extends OpMode {

    DcMotor RightDcMotor;
    DcMotor LeftDcMotor;

    public void init(){
        RightDcMotor = hardwareMap.dcMotor.get("RightMotor");
        LeftDcMotor = hardwareMap.dcMotor.get("LeftMotor");
    }

    public void loop(){
        if(gamepad1.left_stick_y > 0){
            LeftDcMotor.setPower(0.5);
            RightDcMotor.setPower(-0.5);
        } else if (gamepad1.left_stick_y < 0){
            LeftDcMotor.setPower(-0.5);
            RightDcMotor.setPower(0.5);
        } else if(gamepad1.left_stick_x > 0){
            LeftDcMotor.setPower(0.5);
            RightDcMotor.setPower(0.5);
        } else if (gamepad1.left_stick_x < 0){
            LeftDcMotor.setPower(-0.5);
            RightDcMotor.setPower(-0.5);
        } else {
            LeftDcMotor.setPower(0);
            RightDcMotor.setPower(0);
        }

    }
}
