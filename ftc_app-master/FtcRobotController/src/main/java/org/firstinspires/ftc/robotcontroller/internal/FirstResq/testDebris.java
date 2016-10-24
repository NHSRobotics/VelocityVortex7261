package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 7260 on 2/25/2016.
 */
public class testDebris extends OpMode {

    DcMotor chute;
    DcMotor scoop;

    public void init(){
        chute = hardwareMap.dcMotor.get("chute");
        scoop = hardwareMap.dcMotor.get("scoop");
    }

    public void loop(){
        if(gamepad1.right_bumper){
            chute.setPower(1);
        } else if (gamepad1.left_bumper) {
            chute.setPower(-1);
        } else {
            chute.setPower(0);
        }

        if (gamepad1.dpad_down){
            scoop.setPower(1);
        } else if (gamepad1.dpad_up) {
            scoop.setPower(-1);
        } else {
            scoop.setPower(0);
        }
    }
}
