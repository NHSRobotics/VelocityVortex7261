package org.firstinspires.ftc.robotcontroller.internal.FirstResq;

//import com.qualcomm.hardware.ModernRoboticsI2cGyro;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by 7260 on 2/12/2016.
 */
public class CompetitionAutoRedSide7260 extends OpMode{

/**
 * Created by 7260 on 11/20/2015.
 */

    /**
     * Autonomous program that runs to bucket on the red-side, drops the climbers into the bucket, and runs up the mountain
     */

    MovementAuto Chassis;                   //Class that holds  the driving motors
    Servo climberArm;                       //Servo that holds and drops the climbers into the bucket
    Servo leftsideArm;                      //Servo that rotates on the left-side of the chassis to hit the climber levers on the mountain
    Servo rightsideArm;                     //Servo that rotates on the right-side of the chassis to hit the climber levers on the mountain
    DcMotor sweeper;
    UltrasonicSensor eyes;
    ModernRoboticsI2cGyro gyro;             //Gyro sensor that measures angle rotation, allowing for precise turning

    int state = 0;                          //Integer variable that holds the current state for the state machine

    double counts;                          //Double variable that holds the number of counts the encoders have to meet
    int ENCODER_CPR = 1120;                 //The number of counts the encoder counts for one full rotation of the motor
    int wheelDiameter = 4;                  //Wheel diameter used to calculate distance based on encoder count
    int gearRatio = 1;                      //Gear ratio used to calculate distance based on encoder count

    int angleCurrent;                       //Holds the current angle that the robot is facing, found from the gyro
    int angleTarget;                        //Holds the angle that the robot wants to face
    double angleError;                      //The difference between the current angle and target angle
    double drivePower;                      //The power at which the motors spin based upon the angle error - More error = faster spin
    double DRIVING_CONSTANT = .01;          //Constant that scales the angle error to a value that can be set to the drive power
    int i = 0;                              //Count variable

    double SPEED = .5;


    Boolean right = false;

    @Override
    public void init(){
        //
        //Initializes and sets the hardware found through the config file
        //
        Chassis = new MovementAuto(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"),4,1,1120);
        climberArm = hardwareMap.servo.get("climber");
        leftsideArm = hardwareMap.servo.get("leftFlipper");
        rightsideArm = hardwareMap.servo.get("rightFlipper");
        sweeper = hardwareMap.dcMotor.get("scoop");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        //
        //Begins calibration of the gyro, which sets the current direction the robot is facing
        //to 0 degrees turned. The robot should be completely still during this time.
        //
        //Resets the encoder count to zero
        //
        gyro.calibrate();
        Chassis.resetEncoders();

        LegacyModule legacy = hardwareMap.legacyModule.get("legacy");

        legacy.enable9v(5, true);

        eyes = hardwareMap.ultrasonicSensor.get("eyes");

        //
        //Sets the servos to their initialized positions
        //
        climberArm.setPosition(1);
        leftsideArm.setPosition(0);
        rightsideArm.setPosition(.5);
    }

    //
    //State machine that controls the robot during autonomous
    //
    @Override
    public void loop(){

        switch (state){


            //
            //Waits for the gyro to finish calibrating
            //
            case 0:
                Chassis.resetEncoders();
                if(!(gyro.isCalibrating()))
                    state++;
                break;

            //
            //Runs forward 2ft
            //
            case 1:

                sweeper.setPower(-1);
                Chassis.runUsingEncoders();

                //
                //Moves the chassis forward at half SPEED
                //
                Chassis.leftMotor.setPower(SPEED);
                Chassis.rightMotor.setPower(SPEED);

                //
                //Calculates the number of counts the encoders have to reach in order to go 2ft
                //
                counts = ENCODER_CPR * (48 / (Math.PI * wheelDiameter)) * gearRatio;

                //
                //Checks if the motors have ran to 2ft
                //
                //After reaching 2ft, stops the motors and resets the encoders
                //
                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            //Resets  the current time to zero
            //
            //Resets the count to zero
            //
            //Sets the target angle to 45 degrees counterclockwise
            //
            case 2:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    resetStartTime();
                    i = 0;
                    angleTarget = -43;
                    state++;
                }
                break;

            //
            //Rotates the bot 45 degrees in the counterclockwise direction
            //
            case 3:
                Chassis.runUsingEncoders();

                angleCurrent = gyro.getIntegratedZValue();      //Gets the current angle

                angleError = angleCurrent - angleTarget;        //Finds the current difference between the target and current

                drivePower = angleError*DRIVING_CONSTANT;       //Calculates the SPEED based on the current difference

                //
                //Makes sure the drive power doesn't exceed a range that would throw an error
                //
                if (drivePower > 1) {
                    drivePower = 1;
                } else if (drivePower < 0) {
                    drivePower = 0;
                }

                //
                //Sets the chassis to spin counterclockwise
                //
                Chassis.leftMotor.setPower(drivePower);
                Chassis.rightMotor.setPower(-drivePower);

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
                        Chassis.stopDrive();
                        Chassis.resetEncoders();
                        state++;
                    } else{
                        i++;
                    }
                }

                break;

            //
            //Checks to see if the encoders have reset
            //
            case 4:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;

            //
            //Runs forward 7ft
            //
            case 5:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(.2);
                Chassis.rightMotor.setPower(.2);

                counts = ENCODER_CPR * (60 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            case 6:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;

            //
            //Runs backwards 1ft
            //
            case 7:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-SPEED);
                Chassis.rightMotor.setPower(-SPEED);

                counts = ENCODER_CPR * (7 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            //Resets  the current time to zero
            //
            //Resets the count to zero
            //
            //Sets the target angle to 90 degrees counterclockwise
            //
            case 8:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    resetStartTime();
                    angleTarget = -87;
                    i = 0;
                    state++;
                }
                break;

            //
            //Rotates the bot 90 degrees counterclockwise
            //
            case 9:
                Chassis.runUsingEncoders();

                angleCurrent = gyro.getIntegratedZValue();

                angleError = angleCurrent - angleTarget;

                drivePower = angleError*DRIVING_CONSTANT;

                if (drivePower > 1) {
                    drivePower = 1;
                } else if (drivePower < 0) {
                    drivePower = 0;
                }

                Chassis.leftMotor.setPower(drivePower);
                Chassis.rightMotor.setPower(-drivePower);

                if ((gyro.getIntegratedZValue() <= angleTarget) && (getRuntime() > 2)){
                    if ( i == 5) {
                        Chassis.stopDrive();
                        Chassis.resetEncoders();
                        state++;
                    } else{
                        i++;
                    }
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            case 10:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                    sweeper.setPower(0);
                }
                break;

            //
            //Runs forward 23.75 inches
            //
            case 11:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED);
                Chassis.rightMotor.setPower(SPEED);

                counts = ENCODER_CPR * (20 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            case 12:
                if (Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0) {
                    resetStartTime();
                    i = 0;
                    counts = 0;
                    state++;

                }
                break;

            case 13:

                double sonarDistance = eyes.getUltrasonicLevel();

                if (sonarDistance > 18) {

                    counts = ENCODER_CPR * (((sonarDistance - 18) / 2.54) / (Math.PI * wheelDiameter)) * gearRatio;

                    state++;
                } else if (sonarDistance == 0) {
                    break;
                } else {
                    Chassis.resetEncoders();
                    state = state + 2;
                }


                break;

            case 14:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED);
                Chassis.rightMotor.setPower(SPEED);

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;



            case 15:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                    resetStartTime();
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            case 16:

                climberArm.setPosition(0);

                if (getRuntime() > 1.5) {
                    state++;
                    Chassis.resetEncoders();
                    resetStartTime();
                }

                break;

            case 17:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;
            //
            //Runs backwards 23.75 in
            //
            case 18:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(-SPEED);
                Chassis.rightMotor.setPower(-SPEED);

                counts = ENCODER_CPR * (3 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);
                    climberArm.setPosition(1);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            //
            //Checks to see if the encoders have reset
            //
            //Resets  the current time to zero
            //
            //Resets the count to zero
            //
            //Sets the target angle to 135 degrees counterclockwise
            //
            case 19:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    resetStartTime();
                    angleTarget = -170;
                    i = 0;
                    state++;
                }
                break;

            //
            //Rotates 135 degrees counterclockwise
            //
            case 20:
                Chassis.runUsingEncoders();

                angleCurrent = gyro.getIntegratedZValue();

                angleError = angleCurrent - angleTarget;

                drivePower = angleError*DRIVING_CONSTANT;

                if (drivePower > 1) {
                    drivePower = 1;
                } else if (drivePower < 0) {
                    drivePower = 0;
                }

                Chassis.leftMotor.setPower(drivePower);
                Chassis.rightMotor.setPower(-drivePower);

                if ((gyro.getIntegratedZValue() <= angleTarget) && (getRuntime() > 2)){
                    if ( i == 5) {
                        Chassis.stopDrive();
                        Chassis.resetEncoders();
                        state++;
                    } else{
                        i++;
                    }
                }

                break;

            case 21:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    sweeper.setPower(-1);
                    state++;

                }
                break;
            //
            //Runs forward 4ft
            //
            case 22:
                Chassis.runUsingEncoders();

                Chassis.leftMotor.setPower(SPEED);
                Chassis.rightMotor.setPower(SPEED);

                counts = ENCODER_CPR * (25 / (Math.PI * wheelDiameter)) * gearRatio;

                if (Math.abs(Chassis.leftMotor.getCurrentPosition()) >= counts && Math.abs(Chassis.rightMotor.getCurrentPosition()) >= counts) {
                    Chassis.leftMotor.setPower(0);
                    Chassis.rightMotor.setPower(0);
                    sweeper.setPower(0);

                    Chassis.resetEncoders();

                    state++;
                }
                break;

            case 23:
                if(Chassis.leftMotor.getCurrentPosition() == 0 && Chassis.rightMotor.getCurrentPosition() == 0){
                    state++;
                }
                break;


            default:
                break;


        }

        //
        //Shows telemetry data on phone
        //
        //telemetry.addData("Current Angle:", gyro.getIntegratedZValue());
        //telemetry.addData("Difference between Target and Current:", angleError);
        //telemetry.addData("Target Angle:", angleTarget);
        //telemetry.addData("Drive Power", drivePower);

        telemetry.addData("Sonar Distance:", eyes.getUltrasonicLevel());
        telemetry.addData("Encoder Count:", counts);
        telemetry.addData("Current Encoder Count:", Chassis.leftMotor.getCurrentPosition());

        telemetry.addData("State", state);

        //telemetry.addData("Climber Arm Position", climberArm.getPosition());
    }

}
