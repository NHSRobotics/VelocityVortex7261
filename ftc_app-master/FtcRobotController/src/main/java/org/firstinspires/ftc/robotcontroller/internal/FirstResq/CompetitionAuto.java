package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.robotcontroller.internal.general.Movement;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by 7260 on 11/21/2015.
 */
public class CompetitionAuto extends LinearOpModeCamera {

    public void runOpMode() throws InterruptedException {

        Movement Chassis = new Movement(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));

        Chassis.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Chassis.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Chassis.forwardDrive(.25);
        sleep(1000);

        Chassis.leftSpin(.25);
        sleep(1000);

        Chassis.rightSpin(.25);
        sleep(1000);

        Chassis.reverseDrive(.25);
        sleep(1000);

        Chassis.stopDrive();

    }
}
