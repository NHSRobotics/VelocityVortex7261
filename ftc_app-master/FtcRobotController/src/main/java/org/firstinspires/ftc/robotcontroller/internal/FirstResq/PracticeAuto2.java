package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by 7260 on 11/20/2015.
 */
public class PracticeAuto2 extends OpMode{
    MovementAuto Chassis;
    ServoMovementAuto climberArm;
    ServoMovementAuto buttonArm;
    ColorSensor colorSensor;
    GyroSensor gyro;

    int state = 0;
    double counts;
    int ENCODER_CPR = 1680;
    int wheelDiameter = 4;
    int gearRatio = 1;

    @Override
    public void init(){
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);
        climberArm = new ServoMovementAuto(hardwareMap.servo.get("climberArm"));
        buttonArm = new ServoMovementAuto(hardwareMap.servo.get("buttonArm"));
        colorSensor = hardwareMap.colorSensor.get("color");
        gyro = hardwareMap.gyroSensor.get("gyro");

        Chassis.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Chassis.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop(){
        switch (state){
            case 0:
                Chassis.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Chassis.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                state++;
                break;
            case 1:
                Chassis.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Chassis.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                Chassis.leftMotor.setPower(.75);
                Chassis.rightMotor.setPower(.75);

                counts = ENCODER_CPR * (24 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Chassis.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    state++;
                }
                break;
            case 2:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;
            case 3:
                Chassis.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Chassis.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                Chassis.leftMotor.setPower(-.75);
                Chassis.rightMotor.setPower(.75);

                counts = ENCODER_CPR * (4.61 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Chassis.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    state++;
                }
                break;
            case 4:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;
            case 5:
                Chassis.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Chassis.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                Chassis.leftMotor.setPower(.75);
                Chassis.rightMotor.setPower(.75);

                counts = ENCODER_CPR * (68 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Chassis.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    state++;
                }
                break;
            case 6:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;
            case 7:
                Chassis.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Chassis.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                Chassis.leftMotor.setPower(-.75);
                Chassis.rightMotor.setPower(.75);

                counts = ENCODER_CPR * (4.61 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Chassis.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    state++;
                }
                break;
            case 8:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;

                }
                break;
            case 9:
                Chassis.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Chassis.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                Chassis.leftMotor.setPower(.75);
                Chassis.rightMotor.setPower(.75);

                counts = ENCODER_CPR * (23 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Chassis.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    state++;
                }
                break;
            case 10:
            if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                state++;
            }
            break;
            /*case 10:
                if(colorSensor.red() * 255 > )*/
            default:
                break;
        }
    }

}
