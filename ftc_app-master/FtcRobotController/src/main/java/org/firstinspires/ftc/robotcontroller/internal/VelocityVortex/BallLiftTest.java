package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 7260 on 11/2/2016.
 */
public class BallLiftTest extends OpMode {
    CRServo clamp;
    DcMotor lift;

    public void init(){
        clamp = hardwareMap.crservo.get("clamp");
        lift = hardwareMap.dcMotor.get("lift");
    }

    public void loop(){
        if (gamepad1.right_bumper){
            clamp.setDirection(CRServo.Direction.REVERSE);
            clamp.setPower(1);
        } else if (gamepad1.left_bumper) {
            clamp.setDirection(CRServo.Direction.FORWARD);
            clamp.setPower(1);
        } else {
            clamp.setPower(0);
        }

        if (gamepad1.left_stick_y > 0) {
            lift.setPower(.5);
        } else if (gamepad1.left_stick_y < 0){
            lift.setPower(-.5);
        } else {
            lift.setPower(0);
        }
    }
}
