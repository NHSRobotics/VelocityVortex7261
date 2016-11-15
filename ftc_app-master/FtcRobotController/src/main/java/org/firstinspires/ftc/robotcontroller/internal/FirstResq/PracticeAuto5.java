package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.general.MovementAuto;

/**
 * Created by 7260 on 11/20/2015.
 */
public class PracticeAuto5 extends OpMode{
    MovementAuto Chassis;
    Servo climberArm;
    ModernRoboticsI2cGyro gyro;

    int state = 0;

    double counts;
    int ENCODER_CPR = 1120;
    int wheelDiameter = 4;
    int gearRatio = 1;

    int angleCurrent;
    int angleTarget;
    double angleError;
    double drivePower;
    double DRIVING_CONSTANT = .01;
    int i = 0;

    @Override
    public void init(){
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"),4,1,1120);
        climberArm = hardwareMap.servo.get("climber");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();
        Chassis.resetEncoders();

        climberArm.setPosition(0);
    }

    @Override
    public void loop(){


        switch (state){

            case 0:
                Chassis.resetEncoders();
                if(!(gyro.isCalibrating()))
                    state++;
                break;



            case 1:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(.5);
                Chassis.rightMotor.setPower(.5);

                counts = ENCODER_CPR * (24 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;



            case 2:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    resetStartTime();
                    i = 0;
                    angleTarget = -45;
                    state++;
                }
                break;



            case 3:
                    Chassis.runUsingEncoders();

                    angleCurrent = gyro.getIntegratedZValue();

                    angleError = angleCurrent - angleTarget;

                    drivePower = angleError*DRIVING_CONSTANT;

                    if (drivePower > 1) {
                        drivePower = 1;
                    } else if (drivePower < 0) {
                        drivePower = 0;
                    }


                    Chassis.leftMotor.setPower(drivePower);
                    Chassis.rightMotor.setPower(-drivePower);


                if ((gyro.getIntegratedZValue() <= angleTarget) && (getRuntime() > 2)){
                    if ( i == 5) {
                        Chassis.stopDrive();
                        Chassis.resetEncoders();
                        state++;
                    } else{
                        i++;
                    }
                }



                break;

            case 4:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;


            case 5:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(.5);
                Chassis.rightMotor.setPower(.5);

                counts = ENCODER_CPR * (72 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();


                    state++;
                }
                break;
            case 6:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    resetStartTime();
                    angleTarget = -90;
                    i = 0;
                    state++;
                }
                break;

            case 7:
                Chassis.runUsingEncoders();

                    angleCurrent = gyro.getIntegratedZValue();

                    angleError = angleCurrent - angleTarget;

                    drivePower = angleError*DRIVING_CONSTANT;

                    if (drivePower > 1) {
                        drivePower = 1;
                    } else if (drivePower < 0) {
                        drivePower = 0;
                    }

                    Chassis.leftMotor.setPower(drivePower);
                    Chassis.rightMotor.setPower(-drivePower);

                if ((gyro.getIntegratedZValue() <= angleTarget) && (getRuntime() > 2)){
                    if ( i == 5) {
                        Chassis.stopDrive();
                        Chassis.resetEncoders();
                        state++;
                    } else{
                        i++;
                    }
                }

                break;
            case 8:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;

                }
                break;
            case 9:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(.3);
                Chassis.rightMotor.setPower(.3);

                counts = ENCODER_CPR * (23.75 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;
            case 10:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                    resetStartTime();
                }
                break;


            case 11:

                if ((climberArm.getPosition() >= .990) && getRuntime() > 2) {
                    state++;
                    Chassis.resetEncoders();
                }
                else {
                    if ((climberArm.getPosition() + .005) > 1) {
                        climberArm.setPosition(1);
                    } else {
                        climberArm.setPosition(climberArm.getPosition() + .005);
                    }
                }

                break;



            case 12:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;


            case 13:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-.3);
                Chassis.rightMotor.setPower(-.3);

                counts = ENCODER_CPR * (5 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);
                    climberArm.setPosition(0);


                    Chassis.resetEncoders();

                    state++;
                }
                break;

            case 14:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;

            default:
                break;


        }

        telemetry.addData("Current Angle:", gyro.getIntegratedZValue());
        telemetry.addData("Difference between Target and Current:", angleError);
        telemetry.addData("Target Angle:", angleTarget);
        telemetry.addData("Drive Power", drivePower);

        telemetry.addData("Encoder Count:", counts);
        telemetry.addData("Current Encoder Count:", Chassis.leftMotor.getCurrentPosition());

        telemetry.addData("State", state);

        telemetry.addData("Climber Arm Position", climberArm.getPosition());
    }

}
