package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DGTestes extends OpMode {
    DcMotorEx Motor;

    public void init(){
        Motor = hardwareMap.get(DcMotorEx.class, "MET");
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    ()
    public void loop() {
        if (gamepad1.a){
            Motor.setPower(0.7);
        } else if (gamepad1.b) {
            Motor.setPower(0);
        }
    }
}

