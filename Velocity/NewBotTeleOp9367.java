package org.firstinspires.ftc.teamcode.Velocity;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

@TeleOp(name="NewBotTeleOp9367", group="9367")
@Disabled
public class NewBotTeleOp9367 extends OpMode {

    //assigning state variables
    DcMotor frDrive, flDrive, rrDrive, rlDrive, collector, shooter;
    Servo lServo, rServo, ballServo;
  //  GyroSensor gyro;
    ColorSensor colorSensor;
    OpticalDistanceSensor beaconDistanceSensor, lineDistanceSensor;

    long setTime;
    boolean trigger = true;


  //  double lineColorSensorThreshold = 10;
  //  ColorSensor lineColorSensor;

    double slowMotionFactor = 0.3, normalTurningFactor = 0.6, slowMotionTurningFactor = 0.5;
    double left_stick_fl_rr1 = 0, left_stick_fr_rl1 = 0, left_stick_fl_rr2 = 0, left_stick_fr_rl2 = 0;
    double shiftThreshold = 0.7;
    int defaultHeading = 45;
    double shiftDegree = 0;
    double asin = 0, acos = 0;
    double initialHeading = 0;
    double lastHeading = initialHeading;
    double headingCorrectionFactor = 30;
    double rawheadingError = 0;
    double headingError = 0;
    boolean getNewHeading = false;

    double leftX1, leftY1, rightX1, rightY1;
    double leftX2, leftY2, rightX2, rightY2;
    double driver1ControlArray, driver2ControlArray;

    double ballServoRelease = 0.33, ballServoBlock = 0.95;
    double lServoPress = 0.53, lServoRest = 0.89;
    double rServoPress = 0.38, rServoRest = 0;




    @Override
    public void init() {

        // linking variables to hardware components
        flDrive = hardwareMap.dcMotor.get("flDrive");
        frDrive = hardwareMap.dcMotor.get("frDrive");
        rlDrive = hardwareMap.dcMotor.get("rlDrive");
        rrDrive = hardwareMap.dcMotor.get("rrDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        beaconDistanceSensor = hardwareMap.opticalDistanceSensor.get("beaconDistanceSensor");
        lineDistanceSensor = hardwareMap.opticalDistanceSensor.get("lineDistanceSensor");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

   //     gyro = hardwareMap.gyroSensor.get("gyro");

        //gyro.calibrate();
        //gyro.resetZAxisIntegrator();

      //  distanceSensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");
      //  beaconColorSensor = hardwareMap.colorSensor.get("beaconColorSensor");
      //  lineColorSensor = hardwareMap.colorSensor.get("lineColorSensor");

        lServo = hardwareMap.servo.get("lServo");
        rServo = hardwareMap.servo.get("rServo");
        ballServo = hardwareMap.servo.get("ballServo");

        lServo.setPosition(lServoRest);
        rServo.setPosition(rServoRest);
        ballServo.setPosition(ballServoBlock);

        //need to be determined
        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rlDrive.setDirection(DcMotorSimple.Direction.REVERSE);




        //disable the LED light on beacon color sensor
        //beaconColorSensor.enableLed(false);


    }

    @Override
    public void loop() {

        telemetry.addData("lineDistanceSensor", lineDistanceSensor.getLightDetected());
        telemetry.addData("beaconDistanceSensor", beaconDistanceSensor.getLightDetected());
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("green", colorSensor.green());

        telemetry.addData("ballservo", ballServo.getPosition());
        telemetry.addData("lServo", lServo.getPosition());
        telemetry.addData("rServo", rServo.getPosition());



        //trimming the input to avoid noise
        leftX1 = gamepad1.left_stick_x;
        leftY1 = gamepad1.left_stick_y;
        rightX1 = gamepad1.right_stick_x;
        rightY1 = gamepad1.right_stick_y;

        leftX2 = gamepad2.left_stick_x;
        leftY2 = gamepad2.left_stick_y;
        rightX2 = gamepad2.right_stick_x;
        rightY2 = gamepad2.right_stick_y;

        if(Math.abs(leftX1) < 0.5){
            leftX1 = 0;
        }
        if(Math.abs(leftY1) < 0.5){
            leftY1 = 0;
        }
        if(Math.abs(rightX1) < 0.5){
            rightX1 = 0;
        }
        if(Math.abs(rightY1) < 0.5){
            rightY1 = 0;
        }
        if(Math.abs(leftX2) < 0.5){
            leftX2 = 0;
        }
        if(Math.abs(leftY2) < 0.5){
            leftY2 = 0;
        }
        if(Math.abs(rightX2) < 0.5){
            rightX2 = 0;
        }
        if(Math.abs(rightY2) < 0.5){
            rightY2 = 0;
        }





        //With Shift Threshold for driver 1
        if(Math.abs(leftX1) > shiftThreshold){
            left_stick_fl_rr1 = - leftX1;
            left_stick_fr_rl1 = leftX1;
        }
        else if(Math.abs(leftY1) > shiftThreshold){
            left_stick_fl_rr1 = leftY1;
            left_stick_fr_rl1 = leftY1;
        }
        else{
            left_stick_fl_rr1 = leftY1 - leftX1;
            left_stick_fr_rl1 = leftY1 + leftX1;
        }


        //With Shift Threshold for driver 2
        if(Math.abs(leftX2) > shiftThreshold){
            left_stick_fl_rr2 = leftX2;
            left_stick_fr_rl2 = leftX2;
        }
        else if(Math.abs(leftY2) > shiftThreshold){
            left_stick_fl_rr2 = leftY2;
            left_stick_fr_rl2 = - leftY2;
        }
        else{
            left_stick_fl_rr2 = leftX2 + leftY2;
            left_stick_fr_rl2 = leftX2 - leftY2;
        }


        //Slow Motion Mode for both drivers
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            flDrive.setPower(slowMotionFactor * ((left_stick_fl_rr1 + (rightY1 - normalTurningFactor * rightX1)) + (left_stick_fl_rr2 + (rightY2 - normalTurningFactor * rightX2))));
            frDrive.setPower(slowMotionFactor * ((left_stick_fr_rl1 + (rightY1 + normalTurningFactor * rightX1)) + (left_stick_fr_rl2 + (- rightY2 + normalTurningFactor * rightX2))));
            rlDrive.setPower(slowMotionFactor * ((left_stick_fr_rl1 + (rightY1 - normalTurningFactor * rightX1)) + (left_stick_fr_rl2 + (- rightY2 - normalTurningFactor * rightX2))));
            rrDrive.setPower(slowMotionFactor * ((left_stick_fl_rr1 + (rightY1 + normalTurningFactor * rightX1)) + (left_stick_fl_rr2 + (rightY2 + normalTurningFactor * rightX2))));
        }

        //beacon pressing mode for driver 2
        else if(gamepad2.left_bumper){
            flDrive.setPower(- (left_stick_fl_rr2 + (rightY2 + normalTurningFactor * rightX2)));
            frDrive.setPower(- (left_stick_fr_rl2 + (- rightY2 - normalTurningFactor * rightX2)));
            rlDrive.setPower(- (left_stick_fr_rl2 + (- rightY2 + normalTurningFactor * rightX2)));
            rrDrive.setPower(- (left_stick_fl_rr2 + (rightY2 - normalTurningFactor * rightX2)));
        }


        //beacon pressing slow motion
        else if(gamepad2.left_bumper && gamepad2.right_bumper){
            flDrive.setPower(slowMotionFactor * (- (left_stick_fl_rr2 + (rightY2 + normalTurningFactor * rightX2))));
            frDrive.setPower(slowMotionFactor * (- (left_stick_fr_rl2 + (- rightY2 - normalTurningFactor * rightX2))));
            rlDrive.setPower(slowMotionFactor * (- (left_stick_fr_rl2 + (- rightY2 + normalTurningFactor * rightX2))));
            rrDrive.setPower(slowMotionFactor * (- (left_stick_fl_rr2 + (rightY2 - normalTurningFactor * rightX2))));
        }

        //normal mode for both drivers
        else {
            flDrive.setPower((left_stick_fl_rr1 + (rightY1 - normalTurningFactor * rightX1)) + (left_stick_fl_rr2 + (rightY2 - normalTurningFactor * rightX2)));
            frDrive.setPower((left_stick_fr_rl1 + (rightY1 + normalTurningFactor * rightX1)) + (left_stick_fr_rl2 + (- rightY2 + normalTurningFactor * rightX2)));
            rlDrive.setPower((left_stick_fr_rl1 + (rightY1 - normalTurningFactor * rightX1)) + (left_stick_fr_rl2 + (- rightY2 - normalTurningFactor * rightX2)));
            rrDrive.setPower((left_stick_fl_rr1 + (rightY1 + normalTurningFactor * rightX1)) + (left_stick_fl_rr2 + (rightY2 + normalTurningFactor * rightX2)));
        }



        //set collector
        if(gamepad1.right_trigger > 0.5){
            collector.setPower(1);
        }
        else if(gamepad1.left_trigger > 0.5){
            collector.setPower(-1);
        }
        else{
            collector.setPower(0);
        }



        //set shooter control for shooting
        if(gamepad2.right_trigger > 0.5) {
            if (trigger) {
                setTime = System.currentTimeMillis();
                trigger = false;
            }
            else {
                if (System.currentTimeMillis() - setTime > 250 && System.currentTimeMillis() - setTime < 350) {
                    shooter.setPower(0);
                }
                else if (System.currentTimeMillis() - setTime > 350) {
                    shooter.setPower(1.0);
                }
                else {
                    shooter.setPower(-0.15);
                }
            }
        }
        else{
            shooter.setPower(0);
            trigger = true;
        }


        if (gamepad2.x) {
            ballServo.setPosition(ballServoRelease);
            collector.setPower(1);
        } else {
            ballServo.setPosition(ballServoBlock);
            collector.setPower(0);
        }





            }





        }
/*

*/


/*
        //Without Shift Threshold
        //Slow Motion Mode
        if (gamepad1.right_bumper) {
            flDrive.setPower(slowMotionFactor * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_y - slowMotionTurningFactor * gamepad1.right_stick_x)));
            frDrive.setPower(slowMotionFactor * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_y + slowMotionTurningFactor * gamepad1.right_stick_x)));
            rlDrive.setPower(slowMotionFactor * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_y - slowMotionTurningFactor * gamepad1.right_stick_x)));
            rrDrive.setPower(slowMotionFactor * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_y + slowMotionTurningFactor * gamepad1.right_stick_x)));
        }


        //Normal Mode
        else {
            flDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_y - normalTurningFactor * gamepad1.right_stick_x));
            frDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_y + normalTurningFactor * gamepad1.right_stick_x));
            rlDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_y - normalTurningFactor * gamepad1.right_stick_x));
            rrDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_y + normalTurningFactor * gamepad1.right_stick_x));
        }
*/

  //      if(getColorSensorAvg(lineColorSensor) > lineColorSensorThreshold) {
   //         telemetry.addLine("White Line Detected");
   //     }







        //set shooter
    /*    if(gamepad1.a) {

            if (trigger) {
                setTime = System.currentTimeMillis();
                trigger = false;
            }

            else {
                if (System.currentTimeMillis() - setTime > 350 && System.currentTimeMillis() - setTime < 450) {
                    shooter.setPower(0);
                }
                else if (System.currentTimeMillis() - setTime > 450) {
                    shooter.setPower(1.0);
                }
                else {
                    shooter.setPower(-0.15);
                }
            }

        }


        else{
            shooter.setPower(0);
            trigger = true;
        }
*/




//    }
//}
















    /*    //find target degree given input from gamepad stick
        asin = Math.toDegrees(Math.asin(gamepad1.left_stick_y));
        acos = Math.toDegrees(Math.acos(gamepad1.left_stick_x));
        if(asin > 0 && acos > 0){
            shiftDegree = (asin + acos) / 2;
        }
        else if(asin * acos < 0){
            if(asin < 0){
                shiftDegree = asin;
            }
            else{
                shiftDegree = acos;
            }
        }
        else if(asin * acos == 0){
            if(asin == 0){
                shiftDegree = acos;
            }
            else{
                shiftDegree = asin;
            }
        }
        else{
            shiftDegree = acos + 2 * (180 - acos);
        }


        //if gyro is functional
        if(!gyro.isCalibrating()) {
            //rotate on the axis of wheels
            shiftDegree = shiftDegree + gyro.getHeading();

            //heading correction
            rawheadingError = gyro.getHeading() - lastHeading;
            if (Math.abs(rawheadingError) > 180) {
                headingError = rawheadingError - Math.signum(rawheadingError) * 360;
            }


            //spin
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                getNewHeading = true;
            }


            //only shift (not spin)
            if (!getNewHeading) {

                //normal mode
                if (!gamepad1.right_bumper) {
                    flDrive.setPower(Math.cos(Math.toRadians(shiftDegree)) - Math.sin(Math.toRadians(shiftDegree)) - headingError / headingCorrectionFactor);
                    frDrive.setPower(Math.cos(Math.toRadians(shiftDegree)) + Math.sin(Math.toRadians(shiftDegree)) + headingError / headingCorrectionFactor);
                    rlDrive.setPower(Math.cos(Math.toRadians(shiftDegree)) + Math.sin(Math.toRadians(shiftDegree)) - headingError / headingCorrectionFactor);
                    rrDrive.setPower(Math.cos(Math.toRadians(shiftDegree)) - Math.sin(Math.toRadians(shiftDegree)) + headingError / headingCorrectionFactor);
                }

                //slow motion mode
                else {
                    flDrive.setPower(slowMotionFactor * (Math.cos(Math.toRadians(shiftDegree)) - Math.sin(Math.toRadians(shiftDegree)) - headingError / headingCorrectionFactor));
                    frDrive.setPower(slowMotionFactor * (Math.cos(Math.toRadians(shiftDegree)) + Math.sin(Math.toRadians(shiftDegree)) + headingError / headingCorrectionFactor));
                    rlDrive.setPower(slowMotionFactor * (Math.cos(Math.toRadians(shiftDegree)) + Math.sin(Math.toRadians(shiftDegree)) - headingError / headingCorrectionFactor));
                    rrDrive.setPower(slowMotionFactor * (Math.cos(Math.toRadians(shiftDegree)) - Math.sin(Math.toRadians(shiftDegree)) + headingError / headingCorrectionFactor));
                }

            }

            //only spin
           else {
                if (!gamepad1.right_bumper) {
                    flDrive.setPower(gamepad1.right_stick_y - normalTurningFactor * gamepad1.right_stick_x);
                    frDrive.setPower(gamepad1.right_stick_y + normalTurningFactor * gamepad1.right_stick_x);
                    rlDrive.setPower(gamepad1.right_stick_y - normalTurningFactor * gamepad1.right_stick_x);
                    rrDrive.setPower(gamepad1.right_stick_y + normalTurningFactor * gamepad1.right_stick_x);
                } else {
                    flDrive.setPower(slowMotionFactor * (gamepad1.right_stick_y - slowMotionTurningFactor * gamepad1.right_stick_x));
                    frDrive.setPower(slowMotionFactor * (gamepad1.right_stick_y + slowMotionTurningFactor * gamepad1.right_stick_x));
                    rlDrive.setPower(slowMotionFactor * (gamepad1.right_stick_y - slowMotionTurningFactor * gamepad1.right_stick_x));
                    rrDrive.setPower(slowMotionFactor * (gamepad1.right_stick_y + slowMotionTurningFactor * gamepad1.right_stick_x));
                }
                //get new heading
                lastHeading = gyro.getHeading();
                getNewHeading = false;
            }
        }


        //if gyro is not functional
        else{

            if (!gamepad1.right_bumper) {
                flDrive.setPower(gamepad1.right_stick_y - normalTurningFactor * gamepad1.right_stick_x);
                frDrive.setPower(gamepad1.right_stick_y + normalTurningFactor * gamepad1.right_stick_x);
                rlDrive.setPower(gamepad1.right_stick_y - normalTurningFactor * gamepad1.right_stick_x);
                rrDrive.setPower(gamepad1.right_stick_y + normalTurningFactor * gamepad1.right_stick_x);
            } else {
                flDrive.setPower(slowMotionFactor * (gamepad1.right_stick_y - slowMotionTurningFactor * gamepad1.right_stick_x));
                frDrive.setPower(slowMotionFactor * (gamepad1.right_stick_y + slowMotionTurningFactor * gamepad1.right_stick_x));
                rlDrive.setPower(slowMotionFactor * (gamepad1.right_stick_y - slowMotionTurningFactor * gamepad1.right_stick_x));
                rrDrive.setPower(slowMotionFactor * (gamepad1.right_stick_y + slowMotionTurningFactor * gamepad1.right_stick_x));
            }
            //get new heading
            lastHeading = gyro.getHeading();
            getNewHeading = false;

        }


*/
