package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by 7260 on 3/8/2016.
 */
public class testColor extends OpMode {

    ColorSensor color;

    public void init(){
        color = hardwareMap.colorSensor.get("color");
    }

    public void loop(){
        telemetry.addData("Red:", color.red());
        telemetry.addData("Green:", color.green());
        telemetry.addData("Blue:", color.blue());
    }
}
