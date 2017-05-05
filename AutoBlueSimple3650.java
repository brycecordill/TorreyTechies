package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Automagical Blue (Simple)", group = "3650")
public class AutoBlueSimple3650 extends LinearOpMode {
    


    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_3650 hw = new Hardware_3650(hardwareMap);

        waitForStart();

        //sets motors for distance
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move to shooting position
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+1700);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+1700);
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);

        //sleep (wait) so it can finish moving


        //start spinning up shooter

        hw.shooter.setPower(.9);
        Thread.sleep(3300);
        hw.lDrive.setPower(0);
        hw.rDrive.setPower(0);

        //spin up collector
        hw.collector.setPower(-1.00);
        Thread.sleep(750);
        hw.collector.setPower(0);
        Thread.sleep(1500);
        hw.collector.setPower(-1.00);
        Thread.sleep(1500);
        //spin down motors
        hw.collector.setPower(0);
        hw.shooter.setPower(.4);
        Thread.sleep(2000);
        hw.shooter.setPower(0);

        //do a 180
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()-1900);
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+1900);
        hw.rDrive.setPower(.5);
        hw.lDrive.setPower(.5);
        Thread.sleep(3500);

        //ram the ball and park
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()-2700);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()-2700);
        Thread.sleep(5000);



    }
}
