package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "Automagical: REDG", group = "3650")
public class Auto_RED_Gyro extends LinearOpMode{

    ColorSensor colorSensor;
    LightSensor light;
    TouchSensor lTouch, rTouch;
    Servo forePush, aftPush;
    DcMotor lDrive, rDrive, collector, shooter;
    double lThresh, aftNeutral, foreNeutral;
    double initialHeading;

    IMU_class imu;





    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMU_class("imu", hardwareMap);

        lThresh = 0.08; //anything higher is white


        //rest positions for servos
        aftNeutral = .1;
        foreNeutral = 1;

        //button pushing servos
        forePush = hardwareMap.servo.get("forePush");
        aftPush = hardwareMap.servo.get("aftPush");

        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        colorSensor.enableLed(false);
        light = hardwareMap.lightSensor.get("light");

        lTouch = hardwareMap.touchSensor.get("lTouch");
        rTouch = hardwareMap.touchSensor.get("rTouch");

        lDrive.setDirection(DcMotor.Direction.REVERSE);

        //set servos to rest position
        forePush.setPosition(foreNeutral);
        aftPush.setPosition(aftNeutral);

        initialHeading = getHeading(imu);


        waitForStart(); //waits for start button to be pressed

        //sets motors to run with encoder
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move to shooting position
        rDrive.setTargetPosition(rDrive.getCurrentPosition()+1200);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1220);
        rDrive.setPower(.4);
        lDrive.setPower(.4);

        //spin up shooter
        shooter.setPower(.95);

        Thread.sleep(1500);

        //stop and start shooting
        lDrive.setPower(0);
        rDrive.setPower(0);

        //start shooting
        collector.setPower(-1.00);
        Thread.sleep(2500);

        //wait for shooter to speed down
        collector.setPower(0);
        shooter.setPower(.4);
        Thread.sleep(1000);
        shooter.setPower(0);

        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //spin towards left wall
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1550);
        rDrive.setTargetPosition(rDrive.getCurrentPosition()+1550);
        lDrive.setPower(.6);
        rDrive.setPower(.6);

        Thread.sleep(1500);
        turnToAngle(85,imu);

        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive into wall

        //drive fast to save time
        lDrive.setPower(.9);
        Thread.sleep(30);
        rDrive.setPower(.9);
        Thread.sleep(400);

        //use touch sensors to be perpendicular to wall
        lDrive.setPower(.20);
        rDrive.setPower(.20);
        while(!(rTouch.isPressed()) || !(lTouch.isPressed())){
            if(rTouch.isPressed()){
                rDrive.setPower(0);
            }
            else if(lTouch.isPressed()){
                lDrive.setPower(0);
            }

        }
        lDrive.setPower(0);
        rDrive.setPower(0);

        Thread.sleep(500);

        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rDrive.setTargetPosition(rDrive.getCurrentPosition() - 550);
        lDrive.setTargetPosition(lDrive.getCurrentPosition() - 550);
        rDrive.setPower(.4);
        lDrive.setPower(.4);

        Thread.sleep(2000);

        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //spin right to be parallel with beacons
        turnToAngle(-93, imu);
        Thread.sleep(2000);

        //sets motors back to normal mode
        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //drive slowly in order to detect white line
        rDrive.setPower(.16);
        lDrive.setPower(.16);

        //while the line is not detected ...
        while(light.getLightDetected() < lThresh){
            continue;
        }
        //stop once detected
        rDrive.setPower(0);
        lDrive.setPower(0);
        Thread.sleep(1000);

        //detect if red
        if(colorSensor.red() > colorSensor.blue()){
            //hit button with servo
            aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            aftPush.setPosition(aftNeutral);
        }//if not, use other servo
        else if(colorSensor.blue() >= colorSensor.red()){
            //hit button with other servo
            forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            forePush.setPosition(foreNeutral);
        }
        else{
            //run away
        }



        //repeat
        rDrive.setPower(.16);
        lDrive.setPower(.16);
        Thread.sleep(3000);

        while(light.getLightDetected() < .1){
            continue;
        }
        rDrive.setPower(0);
        lDrive.setPower(0);
        Thread.sleep(1000);

        if(colorSensor.red() > colorSensor.blue()){
            //hit button with servo
            aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            aftPush.setPosition(aftNeutral);
        }
        else if(colorSensor.blue() >= colorSensor.red()){
            //hit button with other servo
            forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            forePush.setPosition(foreNeutral);
        }
        else{
            //run away
        }
        Thread.sleep(1000);





    }

    void turnToAngle(double target, IMU_class a){
        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double converted_target;
        initialHeading = getHeading(a);
        converted_target = initialHeading + target;
        double turnError;
        while(Math.abs(converted_target - getHeading(a)) > 1) {
            turnError = converted_target - getHeading(a);
            if(Math.abs(turnError) > 180){
                turnError = turnError - Math.signum(turnError) * 360;
            }
            if(Math.abs(turnError) > 60){
                lDrive.setPower(-Math.signum(turnError) * 0.3);
                rDrive.setPower(Math.signum(turnError) * 0.3);
            }
            //minimum power range: 0.03 - 0.08

            else{
                lDrive.setPower(-Math.signum(turnError) * (0.065 + Math.abs(turnError)/60 * 0.23));
                rDrive.setPower(Math.signum(turnError) * (0.065 + Math.abs(turnError)/60 * 0.23));
            }
            telemetry.addData("degrees to target", Math.abs(getHeading(a) - converted_target));
            telemetry.addData("current heading", getHeading(a));
            telemetry.update();
        }
        lDrive.setPower(0);
        rDrive.setPower(0);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    void turnToAbsoluteAngle(double target, IMU_class a){
        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double converted_target;
        converted_target= initialHeading + target;
        double turnError;
        while(Math.abs(converted_target - getHeading(a)) > 1) {
            turnError = converted_target - getHeading(a);
            if(Math.abs(turnError) > 180){
                turnError = turnError - Math.signum(turnError) * 360;
            }
            if(Math.abs(turnError) > 60){
                lDrive.setPower(-Math.signum(turnError) * 0.3);
                rDrive.setPower(Math.signum(turnError) * 0.3);
            }
            else{
                lDrive.setPower(-Math.signum(turnError) * (0.08 + Math.abs(turnError)/60 * 0.2));
                rDrive.setPower(Math.signum(turnError) * (0.08 + Math.abs(turnError)/60 * 0.2));
            }
            telemetry.addData("degrees to target", Math.abs(getHeading(a) - converted_target));
            telemetry.addData("current heading", getHeading(a));
            telemetry.update();
        }
        lDrive.setPower(0);
        rDrive.setPower(0);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    double getHeading(IMU_class a){
        return a.getAngles()[0];
    }
}
