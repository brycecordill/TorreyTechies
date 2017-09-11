package org.firstinspires.ftc.teamcode.PostSeason17;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by bryce on 3/22/17.
 */
@TeleOp(name = "WenlongBattle", group = "2017")
@Disabled
public class WenlongBattle extends OpMode {

    DcMotor flDrive, frDrive, rlDrive, rrDrive, deathStick;

    @Override
    public void init() {
        flDrive = hardwareMap.dcMotor.get("flDrive");
        frDrive = hardwareMap.dcMotor.get("frDrive");
        rlDrive = hardwareMap.dcMotor.get("rlDrive");
        rrDrive = hardwareMap.dcMotor.get("rrDrive");

        flDrive.setDirection(DcMotor.Direction.REVERSE);
        rlDrive.setDirection(DcMotor.Direction.REVERSE);
        deathStick = hardwareMap.dcMotor.get("deathStick");
    }

    @Override
    public void loop() {

        deathStick.setPower(.7*gamepad1.right_trigger);

        frDrive.setPower(.7*gamepad1.right_stick_y);
        rrDrive.setPower(-.7*gamepad1.right_stick_x);//rr
        flDrive.setPower(-.7*gamepad1.left_stick_x);
        rlDrive.setPower(.7*gamepad1.left_stick_y);//x

    }
}
