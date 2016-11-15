package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.general.MovementAuto;

/**
 * Created by 7260 on 11/9/2015.
 */
public class PracticeAuto extends LinearOpMode {

    MovementAuto Chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);

        //Chassis.forwardDriveAuto(10.0);


        //telemetry.addData("Counts:", Chasis.counts);
    }

}
