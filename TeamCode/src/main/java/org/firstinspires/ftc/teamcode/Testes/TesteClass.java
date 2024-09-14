package org.firstinspires.ftc.teamcode.Testes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous (name = "Sla mn", group = "LinearOpMode")
public class TesteClass extends LinearOpMode {
    DcMotorEx MDF, MDT, MET, MEF;
    IMU imu;
    @Override
    public void runOpMode(){
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");

        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MEF.setDirection(DcMotorSimple.Direction.REVERSE);

        MDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MEF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MET.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

    }
}
