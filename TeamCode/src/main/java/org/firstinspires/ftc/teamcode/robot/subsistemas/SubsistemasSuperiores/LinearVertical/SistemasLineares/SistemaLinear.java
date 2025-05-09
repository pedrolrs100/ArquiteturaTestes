package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes.Condicoes;

public class SistemaLinear  {

    public DcMotorEx motor;
    public Controladorpidf controlador;
    public Condicoes condicoesParada;
    private int ID;
    public static boolean isBusy;
    int targetPosition;
    public SistemaLinear(){}
    public Action GoTo(int target) {

        return new Action() {
            int id = target;
            boolean started = false , condicaoParadaId = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    targetPosition = target;
                    condicoesParada.time.reset();
                    started = true;
                    ID = id;
                    isBusy = true;
                }
                if(ID != id) {
                    reset(motor);
                    isBusy = false;
                    return false;
                };
                controlador.PIDF();

                if(condicoesParada.condicaoParadaFinal()){
                    reset(motor);
                    isBusy = false;
                    return false;
                }
                return true;
            }
        };
    }
    public void reset(DcMotorEx Motor) {
        Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
