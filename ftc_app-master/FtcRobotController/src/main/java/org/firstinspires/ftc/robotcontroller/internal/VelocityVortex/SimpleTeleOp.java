package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.general.Movement;

/**
 * Created by 7260 on 11/10/2016.
 */
public class SimpleTeleOp extends OpMode {

    Movement chassis;
    boolean inverted = false;

    public void init(){
        chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
    }

    public void loop(){
        if (!inverted){
            if (gamepad1.left_stick_y > 0) {
                chassis.forwardDrive(Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.left_stick_y < 0) {
                chassis.reverseDrive(Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.right_stick_x > 0) {
                chassis.rightSpin(1);
            } else if (gamepad1.right_stick_x < 0) {
                chassis.leftSpin(1);
            } else {
                chassis.stopDrive();
            }
        } else {
            if (gamepad1.left_stick_y > 0) {
                chassis.reverseDrive(Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.left_stick_y < 0) {
                chassis.forwardDrive(Math.pow(gamepad1.left_stick_y, 2));
            } else if (gamepad1.right_stick_x > 0) {
                chassis.leftSpin(1);
            } else if (gamepad1.right_stick_x < 0) {
                chassis.rightSpin(1);
            } else {
                chassis.stopDrive();
            }
        }
    }
}
