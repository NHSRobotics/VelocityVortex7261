package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.general.MovementAuto;

//import com.qualcomm.hardware.ModernRoboticsI2cGyro;

/**
 * Created by 7260 on 11/20/2015.
 */
public class PracticeAuto3 extends OpMode{
    MovementAuto Chassis;
    Servo climberArm;
    ModernRoboticsI2cGyro gyro;

    boolean angleMeasure;
    int state = 0;
    int angleCalibrate = 0;
    int angleCorrection;
    double counts;
    int ENCODER_CPR = 1120;
    int wheelDiameter = 4;
    int gearRatio = 1;

    @Override
    public void init(){
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"),4,1,1120);
        climberArm = hardwareMap.servo.get("climber");
        //buttonArm = new ServoMovementAuto(hardwareMap.servo.get("buttonArm"));
        //colorSensor = hardwareMap.colorSensor.get("color");
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
                    state++;
                }
                break;

            case 3:
                Chassis.runUsingEncoders();

                telemetry.addData("angle:", gyro.getIntegratedZValue());
                telemetry.addData("calibrate:", angleCalibrate);
                telemetry.addData("measure:", angleMeasure);

                if(!angleMeasure) {
                    angleCalibrate = gyro.getIntegratedZValue();
                    angleMeasure = true;
                }

                Chassis.leftMotor.setPower(.2);
                Chassis.rightMotor.setPower(-.2);

                if (gyro.getIntegratedZValue() <= (-45)){
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    angleMeasure = false;

                    state++;
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
                    state++;
                }
                break;

            case 7:
                telemetry.addData("angle:", gyro.getIntegratedZValue());
                telemetry.addData("calibrate", angleCalibrate);
                telemetry.addData("measure", angleMeasure);


                Chassis.runUsingEncoders();

                if(angleMeasure == false) {
                    angleCalibrate = gyro.getIntegratedZValue();
                    angleMeasure = true;
                }

                Chassis.leftMotor.setPower(.2);
                Chassis.rightMotor.setPower(-.2);

                if (gyro.getIntegratedZValue() <= (-90 )){
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    angleMeasure = false;

                    state++;
                }
                break;
            case 8:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;

                }
                break;
            case 9:
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
            case 10:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;


            case 11:

                for(double i=0; i <= 1.0; i += .005) {
                    climberArm.setPosition(i);

                }

                if (climberArm.getPosition() >= .995) {
                    state++;
                }
                break;

            case 12:
                Chassis.resetEncoders();
                state++;
                break;

            case 13:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-.5);
                Chassis.rightMotor.setPower(-.5);

                counts = ENCODER_CPR * (10 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) > counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) > counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);


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
    }

}
