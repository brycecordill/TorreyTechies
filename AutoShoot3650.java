package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "Automagically Shoot", group = "3650")
public class AutoShoot3650 extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_3650 hw = new Hardware_3650(hardwareMap);
        
        waitForStart(); //wait for start button

        //move up to firing position
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+1250);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+1250);
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);

        //spin up shooter
        hw.shooter.setPower(.9);

        Thread.sleep(2500);

        hw.lDrive.setPower(0);
        hw.rDrive.setPower(0);

        //start shooting
        hw.collector.setPower(-1.00);
        Thread.sleep(1000);

        //wait for shooter to speed up again
        hw.collector.setPower(0);
        Thread.sleep(750);

        //fire second ball
        hw.collector.setPower(-1.00);
        Thread.sleep(1000);

        //stop collector, spin down shooter
        hw.collector.setPower(0);
        hw.shooter.setPower(.4);
        Thread.sleep(2000);

        hw.shooter.setPower(0);

    }
}
