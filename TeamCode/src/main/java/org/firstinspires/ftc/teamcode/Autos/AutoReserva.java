package org.firstinspires.ftc.teamcode.Testes;

@Autonomous (name = "AutoRes", group = "LinearOpMode")
public class AutoReserva extends LinearOpMode{
    DcMotorEx MEF, MET, MDF, MDT, LSi, LSii;
    Servo garrai, garraii;
    boolean subido = false;
    public void runOpMode(){
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");

        garrai = hardwareMap.get(Servo.class, "garrai");
        garraii = hardwareMap.get(Servo.class, "garraii");

        garraii.setDirection(Servo.Direction.REVERSE);
        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MDT.setDirection(DcMotorSimple.Direction.REVERSE);
        garrai.setPosition(0.5);
        garraii.setPosition(0.5);

        ElapsedTime time = new ElapsedTime;
        time.reset();

        waitForStart();
        time.startTime();
        while (time <= 4){
            MotorsPower(0.3, 0.3, 0.3, 0.3);
        }
        MotorsPower(0, 0, 0, 0);

        time.reset();
        while(time >= 1){
            MotorsPower(0, 0, 0.2, 0.2);
        }
        MotorsPower(0, 0, 0, 0);
        
        time.reset();
        while (time <= 2){
            LSs();
            subindo = true;
        }
        LSs();
        waitSeconds(1);
        garraii.setPosition(0);
        garrai.setDirection(0);
    }
}
public void MotorsPower(double p1, double p2, double p3,double p4) {
    MEF.setPower(p1);
    MDF.setPower(p2);
    MET.setPower(p3);
    MDT.setPower(p4);
}
public viud LSs (){
    double ativo = 0.8;
    double inativo = 0;

    if (!subido){
        LSi.setPower(ativo);
        LSii.setPower(ativo);
    }
    else if (subido == true){
        LSi.setPower(inativo);
        LSii.setPower(inativo);
    }
}