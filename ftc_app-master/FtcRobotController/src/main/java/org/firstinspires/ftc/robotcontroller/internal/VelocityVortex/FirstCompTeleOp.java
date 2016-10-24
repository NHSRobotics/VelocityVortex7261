package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FirstResq.Movement;

/**
 * Created by 7260 on 10/19/2016.
 */
public class FirstCompTeleOp extends OpMode{

    Movement chassis;

    public void init() {
        chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
    }

    public void loop() {
        if (gamepad1.left_stick_y > 0){
            chassis.forwardDrive(Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.left_stick_y < 0){
            chassis.reverseDrive(Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.right_stick_x > 0) {
            chassis.rightSpin(.3*Math.pow(gamepad1.right_stick_x, 2));
        } else if (gamepad1.right_stick_x < 0){
            chassis.leftSpin(.3*Math.pow(gamepad1.right_stick_x, 2));
        } else {
            chassis.stopDrive();
        }
    }
}
