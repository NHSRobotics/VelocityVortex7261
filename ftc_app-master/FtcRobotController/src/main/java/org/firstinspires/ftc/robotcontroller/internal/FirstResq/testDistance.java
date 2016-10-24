package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by 7260 on 2/23/2016.
 */
public class testDistance extends OpMode{

    UltrasonicSensor eyes;

    public void init() {
        LegacyModule legacy = hardwareMap.legacyModule.get("legacy");

        legacy.enable9v(5, true);

        eyes = hardwareMap.ultrasonicSensor.get("eyes");
    }

    public void loop() {
        telemetry.addData("Distance: ", eyes.getUltrasonicLevel());
    }
}
