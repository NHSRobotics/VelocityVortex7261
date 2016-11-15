package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.general.Movement;

/**
 * Created by 7260 on 12/12/2015.
 */
public class SimpleMovement extends OpMode {
    public Movement Chassis;
    Servo ServoArm;
    boolean ToggleServo = false;

    @Override
    public void init() {
        Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        ServoArm = hardwareMap.servo.get("ServoArm");
        ServoArm.setPosition(.7);
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y > 0) {
            Chassis.reverseDrive(Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.left_stick_y < 0) {
            Chassis.forwardDrive(Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.right_stick_x > 0) {
            Chassis.rightSpin(.5 * Math.pow(gamepad1.right_stick_x, 2));
        } else if (gamepad1.right_stick_x < 0) {
            Chassis.leftSpin(.5 * Math.pow(gamepad1.right_stick_x, 2));
        } else {
            Chassis.stopDrive();
        }

        if(gamepad1.a) {
            if (ToggleServo == false) {
                ServoArm.setPosition(.3);
                ToggleServo = true;
            } else if (ToggleServo == true) {
                ServoArm.setPosition(.7);
                ToggleServo = false;
            }
        }
    }
}
