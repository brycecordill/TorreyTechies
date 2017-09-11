package org.firstinspires.ftc.teamcode.PostSeason17;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by bryce on 3/23/17.
 */
@TeleOp(name = "GaranBattle", group = "2017")
@Disabled
public class GaranBattle extends OpMode {

    DcMotor frDrive, flDrive, brDrive, blDrive, meh;
    boolean spin;

    @Override
    public void init() {
        frDrive = hardwareMap.dcMotor.get("frDrive");
        flDrive = hardwareMap.dcMotor.get("flDrive");
        brDrive = hardwareMap.dcMotor.get("brDrive");
        blDrive = hardwareMap.dcMotor.get("blDrive");
        meh = hardwareMap.dcMotor.get("meh");  //spinner

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        blDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        spin = false;
    }

    @Override
    public void loop() {
        frDrive.setPower(gamepad1.right_stick_y);
        brDrive.setPower(gamepad1.right_stick_y);

        flDrive.setPower(gamepad1.left_stick_y);
        blDrive.setPower(gamepad1.left_stick_y);

        if (gamepad1.a && !spin){
            meh.setPower(.8);
            spin = true;
        }
        else if(gamepad1.a && spin){
            meh.setPower(0);
            spin = false;
        }
        else{

        }

    }
}
