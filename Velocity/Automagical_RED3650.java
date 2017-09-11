package org.firstinspires.ftc.teamcode.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "Automagical: RED", group = "3650")
@Disabled
public class Automagical_RED3650 extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_3650 hw = new Hardware_3650(hardwareMap);




        waitForStart(); //waits for start button to be pressed

        //sets motors to run with encoder
        hw.rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move to shooting position
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+1200);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+1220);
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);

        //spin up hw.shooter
        hw.shooter.setPower(.95);

        Thread.sleep(1500);

        //stop and start shooting
        hw.lDrive.setPower(0);
        hw.rDrive.setPower(0);

        //start shooting
        hw.collector.setPower(-1.00);
        Thread.sleep(2500);

        //wait for hw.shooter to speed down
        hw.collector.setPower(0);
        hw.shooter.setPower(.4);
        Thread.sleep(1000);
        hw.shooter.setPower(0);


        //spin towards left wall
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+600);
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+2380);
        hw.lDrive.setPower(.4);
        hw.rDrive.setPower(.4);

        Thread.sleep(3000);

        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive into wall

        //drive fast to save time
        hw.rDrive.setPower(.9);
        hw.lDrive.setPower(.9);
        Thread.sleep(500);

        //use touch sensors to be perpendicular to wall
        hw.lDrive.setPower(.14);
        hw.rDrive.setPower(.14);
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

        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()-400);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()-400);
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);

        Thread.sleep(2000);


        //spin right to be parallel with beacons
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()-970);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+900);
        hw.rDrive.setPower(.3);
        hw.lDrive.setPower(.3);
        Thread.sleep(2000);

        //sets motors back to normal mode
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //drive slowly in order to detect white line
        hw.rDrive.setPower(.16);
        hw.lDrive.setPower(.16);

        //while the line is not detected ...
        while(hw.light.getLightDetected() < hw.lThresh){
            continue;
        }
        //stop once detected
        hw.rDrive.setPower(0);
        hw.lDrive.setPower(0);
        Thread.sleep(1000);

        //detect if red
        if(hw.colorSensor.red() > hw.colorSensor.blue()){
            //hit button with servo
            hw.aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            hw.aftPush.setPosition(hw.aftNeutral);
        }//if not, use other servo
        else if(hw.colorSensor.blue() >= hw.colorSensor.red()){
            //hit button with other servo
            hw.forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            hw.forePush.setPosition(hw.foreNeutral);
        }
        else{
            //run away
        }



        //repeat
        hw.rDrive.setPower(.16);
        hw.lDrive.setPower(.16);
        Thread.sleep(3000);

        while(hw.light.getLightDetected() < .1){
            continue;
        }
        hw.rDrive.setPower(0);
        hw.lDrive.setPower(0);
        Thread.sleep(1000);

        if(hw.colorSensor.red() > hw.colorSensor.blue()){
            //hit button with servo
            hw.aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            hw.aftPush.setPosition(hw.aftNeutral);
        }
        else if(hw.colorSensor.blue() >= hw.colorSensor.red()){
            //hit button with other servo
            hw.forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            hw.forePush.setPosition(hw.foreNeutral);
        }
        else{
            //run away
        }
        Thread.sleep(1000);





    }
}
