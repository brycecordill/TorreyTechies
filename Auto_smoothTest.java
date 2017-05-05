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


@Autonomous(name = "Auto_smoothTest", group = "3650")
public class Auto_smoothTest extends LinearOpMode{

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


        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rDrive.setTargetPosition(rDrive.getCurrentPosition() - 480);
        lDrive.setTargetPosition(lDrive.getCurrentPosition() - 480);
        while(Math.abs(rDrive.getCurrentPosition() - rDrive.getTargetPosition()) < 3) {
            rDrive.setPower(.16 + Math.abs(rDrive.getCurrentPosition() - rDrive.getTargetPosition()) / 480 * 0.24);
            lDrive.setPower(.16 + Math.abs(lDrive.getCurrentPosition() - lDrive.getTargetPosition()) / 480 * 0.24);
        }

        Thread.sleep(2000);

        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
