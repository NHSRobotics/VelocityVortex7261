package org.firstinspires.ftc.robotcontroller.internal.general;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;


/**
 * Created by 7260 on 9/16/2016.
 */
public class MovementAuto extends Movement {

    //private DcMotor leftMotor;              //Declares DcMotor object
    //private DcMotor rightMotor;             //Declares DcMotor object

    int ENCODER_CPR;    //Constant of how many counts/rotation the encoder counts
    private double gearRatio;               //Declares double variable which holds the gear ratio
    private double wheelDiameter;              //Declares int variable which holds the wheel's diameter
    public double counts;
    private double DRIVING_CONSTANT = .01;

    public boolean first;
    private long last;

    public int angleTarget;
    public int angleError;
    public int angleCurrent;
    public double drivePower;
    public int i;

    private double cmDistance;
    private double inDistance;

    /*
        Constructor method that initializes most of the variables and objects
        @param  left            the left side motors
        @param  right           the right side motors
        @param  WheelDiameter   the diameter of the wheel
        @param  GearRatio       the gear ratio
     */
    public MovementAuto(DcMotor left, DcMotor right, double wheelDiameter, double gearRatio, int ENCODER_CPR) {
        super(left, right);

        this.ENCODER_CPR = ENCODER_CPR;
        this.gearRatio = gearRatio;
        this.wheelDiameter = wheelDiameter;

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setDistance(double distance) {
        counts = ENCODER_CPR * (distance / (Math.PI * wheelDiameter)) * gearRatio;
    }

    public boolean checkDistance(double givenCounts) {
        if (Math.abs(leftMotor.getCurrentPosition()) >= givenCounts && Math.abs(rightMotor.getCurrentPosition()) >= givenCounts) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            resetEncoders();

            return true;
        } else {
            return false;
        }
    }

    public void forwardDriveAuto() {
        runUsingEncoders();

        //
        //Moves the chassis forward at half speed
        //
        leftMotor.setPower(.3);
        rightMotor.setPower(.3);
    }

    public boolean backwardDriveAuto(double distance) {
        runUsingEncoders();

        //
        //Moves the chassis forward at half speed
        //
        leftMotor.setPower(-.3);
        rightMotor.setPower(-.3);

        //
        //Calculates the number of counts the encoders have to reach in order to go 2ft
        //
        counts = ENCODER_CPR * (distance / (Math.PI * wheelDiameter)) * gearRatio;

        //
        //Checks if the motors have ran to 2ft
        //
        //After reaching 2ft, stops the motors and resets the encoders
        //
        if (Math.abs(leftMotor.getCurrentPosition()) >= counts && Math.abs(rightMotor.getCurrentPosition()) >= counts) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            resetEncoders();

            return true;
        } else {
            return false;
        }
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean checkEncoderReset(){
        if(leftMotor.getCurrentPosition() == 0 && rightMotor.getCurrentPosition() == 0){
            return true;
        } else {
            return false;
        }
    }

    public void runUsingEncoders() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean calibratingGyro(ModernRoboticsI2cGyro gyro){
        resetEncoders();
        if(first){
            gyro.calibrate();
        }
        if (gyro.isCalibrating()){
            first = false;
        }
        if (!gyro.isCalibrating() && !first){
            return true;
        }
        return false;
    }

    public boolean turnRightAuto(int angle, ModernRoboticsI2cGyro gyro){
        if (first){
            last = System.currentTimeMillis();
            angleTarget = gyro.getIntegratedZValue() + angle;
            i = 0;
            first = false;
        }

        runUsingEncoders();

        angleCurrent = gyro.getIntegratedZValue();

        angleError = angleTarget - angleCurrent;

        drivePower = angleError*DRIVING_CONSTANT;

        if (drivePower > 1) {
            drivePower = 1;
        } else if (drivePower < 0) {
            drivePower = 0;
        }

        leftMotor.setPower(drivePower);
        rightMotor.setPower(-drivePower);

        if ((gyro.getIntegratedZValue() >= angleTarget) || (System.currentTimeMillis() - last > 2)) {
            if ( i == 5) {
                stopDrive();
                first = true;
                resetEncoders();
                return true;
            } else {
                i++;
            }
        } else {
            return false;
        }
        return false;
    }

    public boolean turnLeftAuto(int angle, ModernRoboticsI2cGyro gyro){
        if (first){
            last = System.currentTimeMillis();
            angleTarget = gyro.getIntegratedZValue() - angle;
            i = 0;
            first = false;
        }

        runUsingEncoders();

        angleCurrent = gyro.getIntegratedZValue();

        angleError = angleTarget - angleCurrent;

        drivePower = angleError*DRIVING_CONSTANT;

        if (drivePower > 1) {
            drivePower = 1;
        } else if (drivePower < 0) {
            drivePower = 0;
        }

        leftMotor.setPower(-drivePower);
        rightMotor.setPower(drivePower);

        if ((gyro.getIntegratedZValue() <= angleTarget) || (System.currentTimeMillis() - last > 2)) {
            if ( i == 5) {
                stopDrive();
                first = true;
                resetEncoders();
                return true;
            } else {
                i++;
            }
        } else {
            return false;
        }
        return false;
    }

    public double sonarDistance(UltrasonicSensor eyes){
        cmDistance = eyes.getUltrasonicLevel();
        inDistance = cmDistance / 2.54;
        return inDistance;
    }
}
