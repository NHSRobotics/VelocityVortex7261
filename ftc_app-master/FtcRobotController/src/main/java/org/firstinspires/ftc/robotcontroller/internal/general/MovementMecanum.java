package org.firstinspires.ftc.robotcontroller.internal.general;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by 7260 on 9/21/2016.
 */
public class MovementMecanum {

    /*

    Wheel slant set up

        / \
        \ /
     */

    public DcMotor fR;
    public DcMotor fL;
    public DcMotor bR;
    public DcMotor bL;

    public MovementMecanum(DcMotor fR, DcMotor fL, DcMotor bL, DcMotor bR){
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        this.bL = bL;
        this.bR = bR;
        this.fL = fL;
        this.fR = fR;
    }

    public void stopDrive(){
        fR.setPower(0);
        fL.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void forwardDrive(double power){
        fR.setPower(power);
        fL.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
    }

    public void backwardDrive(double power){
        forwardDrive(-power);
    }

    public void leftDrive(double power){
        fR.setPower(-power);
        fL.setPower(-power);
        bL.setPower(power);
        bR.setPower(power);
    }

    public void rightDrive(double power){
        leftDrive(-power);
    }

    public void rightSpin(double power){
        fR.setPower(power);
        bR.setPower(power);
        fL.setPower(-power);
        bL.setPower(-power);
    }

    public void leftSpin(double power){
        rightSpin(-power);
    }
}
