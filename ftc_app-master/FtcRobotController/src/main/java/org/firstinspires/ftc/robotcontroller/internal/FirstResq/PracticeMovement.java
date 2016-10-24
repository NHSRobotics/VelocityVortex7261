package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by phama21 on 9/13/2015.
 */
public class PracticeMovement extends OpMode {

    public Movement Chassis;
    public ServoMovement ArmT;
    public ServoMovement ArmB;

    public boolean top;

    //Sets up the robot by finding the motor controller equipment
    @Override
    public void init() {
        Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        ArmT = new ServoMovement(hardwareMap.servo.get("armT"));
        ArmB = new ServoMovement(hardwareMap.servo.get("armB"));
        top = true;
    }

    //Simple movement that utilizes the buttons for precise movement and joysticks for general movement
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
        } else if (gamepad1.dpad_left) {
            if(top){
                ArmT.posIncrease();
            } else {
                ArmB.posIncrease();
            }
        } else if (gamepad1.dpad_right) {
            if(top) {
                ArmT.posDecrease();
            } else {
                ArmB.posDecrease();
            }
        } else if (gamepad1.a) {
            if (top) {
                top = false;
            } else {
                top = true;
            }
        } else {
            Chassis.stopDrive();
        }
    }
}