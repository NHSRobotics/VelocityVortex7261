package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 7260 on 2/3/2016.
 */
public class SettingServos extends OpMode{

    Servo leftsideArm;
    Servo rightsideArm;

    @Override
    public void init() {

        leftsideArm = hardwareMap.servo.get("leftsideArm");
        rightsideArm = hardwareMap.servo.get("rightsideArm");

        leftsideArm.setPosition(0);
        rightsideArm.setPosition(0);


    }

    @Override
    public void loop() {

        leftsideArm.setPosition(.58);
        rightsideArm.setPosition(.62);

        /*
        if ((leftsideArm.getPosition() + .005) >= 1) {
            leftsideArm.setPosition(1);
            if ((rightsideArm.getPosition() + .005) > 1) {
                rightsideArm.setPosition(1);
            } else {
                rightsideArm.setPosition(rightsideArm.getPosition() + .005);
            }
        } else {
            leftsideArm.setPosition(leftsideArm.getPosition() + .005);
        }
        telemetry.addData("Left-Side Servo Position", leftsideArm.getPosition());
        telemetry.addData("Right-Side Servo Position", rightsideArm.getPosition());
        */

    }
}
