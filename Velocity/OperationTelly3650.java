package org.firstinspires.ftc.teamcode.Velocity;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;




@TeleOp(name="Operation: Telly", group="3650")
@Disabled
public class OperationTelly3650 extends OpMode {

    Hardware_3650 hw;





    @Override
    public void init() {

        hw = new Hardware_3650(hardwareMap);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {

        //set motor controls to joystick
        hw.lDrive.setPower(-(gamepad1.left_stick_y*.7));
        hw.rDrive.setPower(-(gamepad1.right_stick_y*.7));

        //code for beacon pushers
        if(gamepad1.right_bumper){
            hw.forePush.setPosition(0);
        }
        else if(gamepad1.left_bumper){
            hw.aftPush.setPosition(1.00);
        }
        else{
            hw.aftPush.setPosition(hw.aftNeutral);
            hw.forePush.setPosition(hw.foreNeutral);
        }

        //shooter code
        if (gamepad2.dpad_down && gamepad2.right_trigger == 0){
            hw.shooter.setPower(-1.0);
        }
        else if (!gamepad2.dpad_down && gamepad2.right_trigger > 0){
            hw.shooter.setPower(gamepad2.right_trigger);
        }
        else{
            hw.shooter.setPower(0);
        }





        //collector controls
        if (gamepad2.left_bumper){
            hw.collector.setPower(-1.0);
        }
        else if(gamepad2.right_bumper){
            hw.collector.setPower(1.00);
        }
        else {
            hw.collector.setPower(0);
        }


        //values to be shown on the driver station
        telemetry.addData("Light", hw.light.getLightDetected());
        telemetry.addData("Red ", hw.colorSensor.red());
        telemetry.addData("Green ", hw.colorSensor.green());
        telemetry.addData("Blue ", hw.colorSensor.blue());
        telemetry.addData("Aft ", hw.aftPush.getPosition());
        telemetry.addData("Fore ", hw.forePush.getPosition());
    }
}
