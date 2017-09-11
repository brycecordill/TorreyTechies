package org.firstinspires.ftc.teamcode.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by bryce on 2/25/17.
 */

@Autonomous(name = "Automagical BlueG", group = "3650")
@Disabled
public class Auto_Blue_Gyro extends LinearOpMode {

    Hardware_3650 hw;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware_3650(hardwareMap);

        waitForStart(); //waits for start button to be pressed

        //sets motors to run with encoder
        hw.lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move to shooting position

        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+1300);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+1320);
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);

        //spin up shooter
        hw.shooter.setPower(.95);

        Thread.sleep(2000);

        //stop and start shooting
        hw.lDrive.setPower(0);
        hw.rDrive.setPower(0);

        //start shooting
        hw.collector.setPower(-1.00);
        Thread.sleep(2500);

        //wait for shooter to speed down
        hw.collector.setPower(0);
        hw.shooter.setPower(.4);
        Thread.sleep(1000);
        hw.shooter.setPower(0);

        hw.rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //spin towards right wall
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+1450);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+1450);
        hw.lDrive.setPower(.6);
        hw.rDrive.setPower(.6);

        Thread.sleep(1500);

        turnToAngle(-90,hw.imu);

        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive into wall (almost)

        //drive fast to reduce time
        hw.rDrive.setPower(.9);
        hw.lDrive.setPower(.9);
        Thread.sleep(500);

        //use touch sensors to be perpendicular to wall
        hw.lDrive.setPower(.22);
        hw.rDrive.setPower(.22);
        while(!(hw.rTouch.isPressed()) || !(hw.lTouch.isPressed())){
            if(hw.rTouch.isPressed()){
                hw.rDrive.setPower(0);
            }
            else if(hw.lTouch.isPressed()){
                hw.lDrive.setPower(0);
            }

        }
        hw.lDrive.setPower(0);
        hw.rDrive.setPower(0);

        Thread.sleep(500);

        hw.rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()-450);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()-450);
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);

        Thread.sleep(2000);

        //spin right to be parallel with beacons
        /*hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition() - 970);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition() + 900);
        hw.rDrive.setPower(.3);
        hw.lDrive.setPower(.3);
        Thread.sleep(1500);*/

        turnToAngle(270,hw.imu);

        //sets motors back to normal mode
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //drive slowly in order to detect white line
        hw.rDrive.setPower(-.16);
        hw.lDrive.setPower(-.16);

        //while the line is not detected ...
        while (hw.light.getLightDetected() < hw.lThresh) {
            continue;
        }
        //stop once detected
        hw.rDrive.setPower(0);
        hw.lDrive.setPower(0);
        Thread.sleep(200);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+100);
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+100);
        hw.rDrive.setPower(.3);
        hw.lDrive.setPower(.3);

        Thread.sleep(1000);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rDrive.setPower(0);
        hw.lDrive.setPower(0);

        //detect if red
        if (hw.colorSensor.red() < hw.colorSensor.blue()) {
            //hit button with servo
            hw.aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            hw.aftPush.setPosition(hw.aftNeutral);
        }//if not, use other servo
        else if (hw.colorSensor.blue() <= hw.colorSensor.red()) {
            //hit button with other servo
            hw.forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            hw.forePush.setPosition(hw.foreNeutral);
        } else {
            //run away
        }


        //repeat
        hw.rDrive.setPower(-.16);
        hw.lDrive.setPower(-.16);
        Thread.sleep(1500);

        while (hw.light.getLightDetected() < hw.lThresh) {
            continue;
        }
        hw.rDrive.setPower(0);
        hw.lDrive.setPower(0);
        Thread.sleep(200);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+100);
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+100);
        hw.rDrive.setPower(.3);
        hw.lDrive.setPower(.3);

        Thread.sleep(1000);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rDrive.setPower(0);
        hw.lDrive.setPower(0);

        if (hw.colorSensor.red() < hw.colorSensor.blue()) {
            //hit button with servo
            hw.aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            hw.aftPush.setPosition(hw.aftNeutral);
        } else if (hw.colorSensor.blue() <= hw.colorSensor.red()) {
            //hit button with other servo
            hw.forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            hw.forePush.setPosition(hw.foreNeutral);
        } else {
            //run away
        }
        Thread.sleep(1000);

    }

    void turnToAngle(double target, IMU_class a){
        hw.lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double converted_target;
        hw.initialHeading = hw.getHeading(a);
        converted_target = hw.initialHeading + target;
        double turnError;
        while(Math.abs(converted_target - hw.getHeading(a)) > 1) {
            turnError = converted_target - hw.getHeading(a);
            if(Math.abs(turnError) > 180){
                turnError = turnError - Math.signum(turnError) * 360;
            }
            else if(Math.abs(turnError) > 60){
                hw.lDrive.setPower(-Math.signum(turnError) * 0.3);
                hw.rDrive.setPower(Math.signum(turnError) * 0.3);
            }
            //minimum power range: 0.03 - 0.08

            else{
                hw.lDrive.setPower(-Math.signum(turnError) * (0.065 + Math.abs(turnError)/60 * 0.23));
                hw.rDrive.setPower(Math.signum(turnError) * (0.065 + Math.abs(turnError)/60 * 0.23));
            }
            telemetry.addData("degrees to target", Math.abs(hw.getHeading(a) - converted_target));
            telemetry.addData("current heading", hw.getHeading(a));
            telemetry.update();
        }
        hw.lDrive.setPower(0);
        hw.rDrive.setPower(0);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
