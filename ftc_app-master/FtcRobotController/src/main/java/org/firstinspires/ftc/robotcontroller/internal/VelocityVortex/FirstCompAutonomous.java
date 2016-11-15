package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.general.MovementAuto;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by 7260 on 10/28/2016.
 */
public class FirstCompAutonomous extends OpMode {
    MovementAuto chassis;
    int state = 0;
    double counts = 0;

    public void init(){
        chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);
        chassis.resetEncoders();
    }

    public void loop(){
        switch (state){
            case 0:
                chassis.resetEncoders();
                state++;
                break;

            case 1:
                chassis.runUsingEncoders();

                chassis.reverseDrive(.6);

                counts = 1120 * (56 / (Math.PI * 4));

                if (Math.abs(chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(chassis.rightMotor.getCurrentPosition()) >= counts) {
                    chassis.leftMotor.setPower(0);
                    chassis.rightMotor.setPower(0);

                    chassis.resetEncoders();

                    state++;
                }
                break;

            default:
                break;

        }
    }
}
