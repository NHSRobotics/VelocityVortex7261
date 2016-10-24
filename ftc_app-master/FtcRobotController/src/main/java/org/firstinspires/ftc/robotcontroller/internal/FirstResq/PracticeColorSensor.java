package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by 7260 on 11/17/2015.
 */
public class PracticeColorSensor extends OpMode{
    public ColorSensor sensorRGB;
    int rb;

    @Override
    public void init() {
        sensorRGB = hardwareMap.colorSensor.get("color");
        rb = 0;
    }

    @Override
    public void loop() {
        double red = sensorRGB.red();
        double green = sensorRGB.green();
        double blue = sensorRGB.blue();

        if(red > blue){
            rb++;
        }

        telemetry.addData("R: ", red);
        telemetry.addData("G: ", green);
        telemetry.addData("B: ", blue);
        telemetry.addData("Red > Blue", rb);
           // DbgLog.msg("R:" + red + " G:" + green + " B:" + blue);
    }
}
