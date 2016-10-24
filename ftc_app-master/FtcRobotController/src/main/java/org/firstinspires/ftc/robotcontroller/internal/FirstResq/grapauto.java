package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by 7260 on 1/30/2016.
 */
public class grapauto extends OpMode{
    MovementAuto Chassis;

    int state = 0;
    double counts;
    int ENCODER_CPR = 1680;
    int wheelDiameter = 4;
    int gearRatio = 1;

    @Override
    public void init() {
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);

        Chassis.resetEncoders();
    }

    @Override
    public void loop() {
        switch (state){
            case 1:
                if (Chassis.leftMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;
            case 2:
                Chassis.runUsingEncoders();

                Chassis.rightMotor.setPower(.5);
                Chassis.leftMotor.setPower(.5);

                counts = ENCODER_CPR * (55 / (Math.PI * wheelDiameter)) * gearRatio;

                if(Chassis.leftMotor.getCurrentPosition() > counts){
                    Chassis.resetEncoders();

                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    state++;
                }
                break;
            default:
                break;
        }


    }
}
