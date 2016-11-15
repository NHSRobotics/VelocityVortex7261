package org.firstinspires.ftc.robotcontroller.internal.VelocityVortex;

import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcontroller.internal.general.MovementAuto;

import java.security.spec.ECField;

import for_camera_opmodes.LinearOpModeCamera;
import for_camera_opmodes.OpModeCamera;

/**
 * Created by 7260 on 10/31/2016.
 */
public class CameraAutonomous extends LinearOpModeCamera {
    MovementAuto chassis;
    DcMotor leftShooter;
    DcMotor rightShooter;
    DcMotor lift;
    ModernRoboticsI2cGyro gyro;

    int WHEEL_DIAMETER = 4;
    int ENCODER_CPR = 1120;
    double SPEED = .4;
    double counts = 0;

    int redValue = 0;
    int blueValue = 0;
    int greenValue = 0;

    int i = 0;
    int angleTarget;
    int angleCurrent;
    double angleError;
    double drivePower;
    double DRIVING_CONSTANT = .01;

    int ds2 = 2;
    int state = 0;

    public void runOpMode() throws InterruptedException {

        chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), 4, 1, 1120);
        //leftShooter = hardwareMap.dcMotor.get("leftshoot");
        //rightShooter = hardwareMap.dcMotor.get("rightshoot");
        //lift = hardwareMap.dcMotor.get("second");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        //leftShooter.setDirection(DcMotor.Direction.REVERSE);

        if (isCameraAvailable()) {
            setCameraDownsampling(8);
            telemetry.addData("camera is available", true);
            startCamera();

            waitForStart();

            try {
                while (opModeIsActive()){
                    switch (state) {
                        case 0:
                            if (!gyro.isCalibrating()){
                                state++;
                            }
                            break;

                        case 1:
                            chassis.runUsingEncoders();

                            chassis.forwardDrive(SPEED);

                            counts = ENCODER_CPR * (60 / (Math.PI * WHEEL_DIAMETER));

                            if (chassis.checkDistance(counts)){
                                chassis.resetEncoders();
                                state++;
                            }
                            break;

                        case 2:
                            if(chassis.checkEncoderReset()){
                                resetStartTime();
                                i = 0;
                                angleTarget = -47;
                                state++;
                            }
                            break;

                        case 3:
                            chassis.runUsingEncoders();

                            angleCurrent = gyro.getIntegratedZValue();      //Gets the current angle

                            angleError = Math.abs(angleTarget - angleCurrent);        //Finds the current difference between the target and current

                            drivePower = angleError*DRIVING_CONSTANT;       //Calculates the speed based on the current difference

                            //
                            //Makes sure the drive power doesn't exceed a range that would throw an error
                            //
                            if (drivePower > 1) {
                                drivePower = 1;
                            } else if (drivePower < 0) {
                                drivePower = 0;
                            }

                            //
                            //Sets the chassis to spin clockwise
                            //
                            chassis.leftSpin(drivePower);
                            //
                            //Checks to see if the bot has reached or exceed its angle target
                            //
                            //Also, it checks if the code has gotten stuck in a loop. Proceeds if it has taken longer than 2 seconds
                            //
                            //Count is incremented to 5 to allow for small delay
                            //
                            //Stops the motors and resets the encoders
                            //
                            if ((gyro.getIntegratedZValue() <= angleTarget) && (getRuntime() > 2)){
                                if ( i == 5) {
                                    chassis.stopDrive();
                                    chassis.resetEncoders();
                                    state++;
                                } else{
                                    i++;
                                }
                            }
                            break;

                        case 4:
                            if (imageReady()) { // only do this if an image has been returned from the camera
                                // get image, rotated so (0,0) is in the bottom left of the preview window
                                Bitmap rgbImage;
                                rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

                                for (int x = (int)(rgbImage.getWidth() * .54); x < (int)(rgbImage.getWidth()* .77); x++) {
                                    for (int y = (int)(rgbImage.getHeight() * .09); y < (int)(rgbImage.getHeight() * .18); y++) {
                                        int pixel = rgbImage.getPixel(x, y);
                                        redValue += red(pixel);
                                        blueValue += blue(pixel);
                                        greenValue += green(pixel);
                                    }
                                }
                                state++;
                            }
                            break;

                        default:
                            break;
                    }
                    telemetry.addData("State: ", state);
                    telemetry.addData("Red: ", redValue);
                    telemetry.addData("Blue: ", blueValue);
                    telemetry.addData("Green: ", greenValue);
                }
            } catch (Exception e) {
                stopCamera();
            }

        }
    }
}
