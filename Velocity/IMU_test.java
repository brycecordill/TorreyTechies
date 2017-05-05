package org.firstinspires.ftc.teamcode.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "IMU_test", group = "3650")
@Disabled
public class IMU_test extends LinearOpMode{

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

        turnToAngle(90, imu);




    }

    void turnToAngle(double target, IMU_class a){
        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double converted_target;
        initialHeading = getHeading(a);
        converted_target= initialHeading + target;
        double turnError;
        while(Math.abs(converted_target - getHeading(a)) > 3) {
            turnError = converted_target - getHeading(a);
            if(Math.abs(turnError) > 180){
                turnError = turnError - Math.signum(turnError) * 360;
            }
            if(Math.abs(turnError) > 30){
                lDrive.setPower(-Math.signum(turnError) * 0.3);
                rDrive.setPower(Math.signum(turnError) * 0.3);
            }
            else{
                lDrive.setPower(-Math.signum(turnError) * (0.06 + turnError/30 * 0.24));
                rDrive.setPower(Math.signum(turnError) * (0.06 + turnError/30 * 0.24));
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
