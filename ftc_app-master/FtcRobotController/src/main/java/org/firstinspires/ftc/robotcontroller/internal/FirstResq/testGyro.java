package org.firstinspires.ftc.robotcontroller.internal.FirstResq;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by 7260 on 1/15/2016.
 */
public class testGyro extends OpMode{
    Movement Chassis;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void init() {
        Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    @Override
    public void loop() {
        //telemetry.addData("Head:", gyro.getHeading());
        telemetry.addData("isCalibrating:", gyro.isCalibrating());
        telemetry.addData("Integrated:", gyro.getIntegratedZValue());


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


    }
}
