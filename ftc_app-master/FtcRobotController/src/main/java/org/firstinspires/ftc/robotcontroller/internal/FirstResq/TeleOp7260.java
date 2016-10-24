package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by 7260 on 1/30/2016.
 */
public class TeleOp7260 extends OpMode{
    public Movement Chassis;
    ServoMovement climber;


    @Override
    public void init() {
        Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        //climber = new ServoMovement(hardwareMap.servo.get("climber"));
        //chute = hardwareMap.dcMotor.get("chute");
        //scoop = hardwareMap.dcMotor.get("scoop");
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y < 0) {
            Chassis.reverseDrive(.5 * Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.left_stick_y > 0) {
            Chassis.forwardDrive(.5 * Math.pow(gamepad1.left_stick_y, 2));
        } else if (gamepad1.right_stick_x < 0) {
            Chassis.rightSpin(.5 * Math.pow(gamepad1.right_stick_x, 2));
        } else if (gamepad1.right_stick_x > 0) {
            Chassis.leftSpin(.5 * Math.pow(gamepad1.right_stick_x, 2));
        } else {
            Chassis.stopDrive();
        }

        if(gamepad1.a){
            climber.posIncrease();
        } else if (gamepad1.b) {
            climber.posDecrease();
        }
    }
}
