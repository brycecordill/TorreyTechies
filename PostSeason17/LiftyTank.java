package org.firstinspires.ftc.teamcode.PostSeason17;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by bryce on 5/5/17.
 */
@TeleOp(name = "LiftyTank", group = "2017")
@Disabled
public class LiftyTank extends OpMode{
    DcMotor rTrack, lTrack, elevator, flag;

    @Override
    public void init() {
        rTrack = hardwareMap.dcMotor.get("rTrack");
        lTrack = hardwareMap.dcMotor.get("lTrack");
        lTrack.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator = hardwareMap.dcMotor.get("elevator");
        flag = hardwareMap.dcMotor.get("flag");

    }

    @Override
    public void loop() {
        lTrack.setPower(gamepad1.left_stick_y);
        rTrack.setPower(gamepad1.right_stick_y);
        if(gamepad1.left_trigger > 0){
            elevator.setPower(-(gamepad1.left_trigger));
        }
        else if(gamepad1.right_trigger > 0){
            elevator.setPower(gamepad1.right_trigger);
        }
        else{
            elevator.setPower(0);
        }
        if (gamepad1.a){
            flag.setPower(1);
        }



    }
}
