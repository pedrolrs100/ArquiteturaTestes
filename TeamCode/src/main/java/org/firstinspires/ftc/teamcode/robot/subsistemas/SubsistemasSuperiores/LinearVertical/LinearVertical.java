package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.common.PIDTargetChecker;

import java.util.HashMap;
@Config
public class LinearVertical {


    public final DcMotorEx motorR;
    public final DcMotorEx motorL;
    LinearVerticalStates linearVerticalstate = LinearVerticalStates.Initial;
    public double timeElevadorGoTo = 0;
    // controla descer
    public boolean needToChangeTarget = false;
    public double timeToChangeTarget = 0;
    public int wantedTarget = 0;
    public static boolean monitor= false, isBusy = false,hang = false;
    HashMap<LinearVerticalStates, Integer> mapLinearVertical = new HashMap<>();
    public static int portaLinearVerticalDireita, portaLinearVerticalEsquerdo, margem = 30, margemAut = 150 , sense = 40, ID = 0,readyHangPos2 =3100  ,readyHangPos = 1950, hangPos = -100000;
    public static double tempoParaEstabilizacao = 0.2, correnteLimite = 2.4;
    public PIDTargetChecker pidTargetChecker = new PIDTargetChecker(margem, tempoParaEstabilizacao);
    public PIDTargetChecker pidTargetCheckerAut = new PIDTargetChecker(margemAut, tempoParaEstabilizacao);
    public ElapsedTime tempoIndoAteOsetPoint = new ElapsedTime();
    public int position;
    public double power;
    public static int targetPosition = 0;
    public static  int alturaOuttakeChamber = 546;
    public static double p = 0.007, i = 0, d = 0.000,f = 0.8, ll = 0, kll = 0,valorDeSurtoDeCorrente = 3.8, valorDeSurtoDeCorrenteTopo = 0.9;
    PIDController controller = new PIDController(p, i, d);

    /* POSIÇÕES PRESETS */
    private int lowBasketPos = 3500, drivingPos = 200, intakingPos = 10;

    // todo: criar botão para reset
    public LinearVertical (HardwareMap hardwareMap) {


        this.motorL =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalL);
        this.motorR =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalR);
        this.power = motorR.getPower();
        portaLinearVerticalDireita = motorR.getPortNumber();
        portaLinearVerticalEsquerdo = motorL.getPortNumber();


        this.motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.reset();
        this.position = motorR.getCurrentPosition();
        this.targetPosition = this.position;
        mapLinearVertical.put(LinearVerticalStates.Outtake,1000);
    }

    public void reset() {

        this.motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        this.motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public double PIDF() {

        double kp = p;

        int linearpos = motorR.getCurrentPosition();


        if(linearpos < this.targetPosition) {
            kp = p * 8;
        }
        if(linearpos > this.targetPosition + 200){
            kp = p * 8;
        }

        if(motorR.getCurrent(CurrentUnit.AMPS) > valorDeSurtoDeCorrenteTopo && targetPosition > 2700 && motorR.getCurrentPosition() > 2600) {
            targetPosition -= 20;
            motorR.setPower(0.4);
        }
        if ((motorR.getCurrent(CurrentUnit.AMPS) > valorDeSurtoDeCorrente && targetPosition < 1800 && targetPosition > 0) ) {
            targetPosition = (targetPosition + motorR.getCurrentPosition()) / 2; // Aproxima suavemente do valor atual
        }
        if(this.motorR.getCurrentPosition() < 0 && targetPosition < 10 && targetPosition > -180) {
            motorR.setPower(0);
            motorL.setPower(0);
            return controller.getPositionError();
        }

        controller.setPID(kp, i,d);
        double pid = controller.calculate(linearpos, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition)) * f;
        if(hang){
            motorR.setPower(-1);
            motorL.setPower(-1);
            return pid + ff;
        }
        motorL.setPower(pid+ff);
        motorR.setPower(pid+ff);


        return pid + ff;
    }
    public void upSetPoint() {
        changeTarget(targetPosition + sense);
    }
    public void downSetPoint() {
        changeTarget(targetPosition - sense);
    }
    public void changeTarget(int target) {

        targetPosition = target;
        tempoIndoAteOsetPoint.reset();
    }

    public Action ElevadorGoTo(int target) {

        return new Action() {
            int id = target;
            ElapsedTime time = new ElapsedTime();
            boolean started = false;
            int topo = 2650;
            boolean condicaoDeParada, condicaoParadaSurtoEnergia = false, condicaoDeParadaId = false, condicaoParadaTopo = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    targetPosition = target;
                    time.reset();
                    started = true;
                    ID = id;
                    isBusy = true;

                }

                PIDF();
                telemetryPacket.addLine("Linear pos:"+motorR.getCurrentPosition()+" | Linear Alvo"+targetPosition);
                timeElevadorGoTo = time.time();

                if(target > 0) {
                    condicaoDeParada = chegouNoTargetAut();
                }
                else {
                    condicaoDeParada = motorR.getCurrentPosition() >= targetPosition - margemAut  &&  motorR.getCurrentPosition() <= targetPosition + margemAut ;
                    condicaoParadaSurtoEnergia = (motorR.getCurrent(CurrentUnit.AMPS) >= correnteLimite);
                }

                //condicaoParadaSurtoEnergia = (motorR.getCurrent(CurrentUnit.AMPS) >= correnteLimite) && !hang;
                condicaoDeParadaId = ID != id;
                condicaoParadaTopo = motorR.getCurrentPosition() >= topo;
                if(condicaoParadaSurtoEnergia && time.time() > 0.1 && motorR.getCurrentPosition() < 230) {
                    motorL.setPower(Math.cos(Math.toRadians(target)) * f);
                    motorR.setPower(Math.cos(Math.toRadians(target)) * f);
                    reset();
                    isBusy = false;
                    return false;
                }
                if(condicaoDeParada || condicaoDeParadaId || condicaoParadaTopo) {
                    motorL.setPower(Math.cos(Math.toRadians(target)) * f);
                    motorR.setPower(Math.cos(Math.toRadians(target)) * f);
                    isBusy = false;
                    return false;
                }


                return true;
            }

        };

    }

    public Action goToReadyHang() {
        return ElevadorGoTo(readyHangPos);
    }
    public Action goToReadyHang2() {
        return ElevadorGoTo(readyHangPos2);
    }

    public Action goToHang() {
        return ElevadorGoTo(hangPos);
    }
    public boolean chegouNoTarget() {
        return pidTargetChecker.hasReachedTarget(targetPosition, motorR.getCurrentPosition());
    }
    public boolean chegouNoTargetAut() {
        return pidTargetCheckerAut.hasReachedTarget(targetPosition, motorR.getCurrentPosition());
    }



    public void monitor(Telemetry telemetry) {
        if (monitor) {

            telemetry.addData("VERTICAL-Posição Motor Left: ",motorL.getCurrentPosition());
            telemetry.addData("VERTICAL-Posição Motor Right: ",motorR.getCurrentPosition());
            telemetry.addData("VERTICAL-alvo: ",targetPosition);
            telemetry.addData("VERTICAL-estado hang: ",hang);
            telemetry.addData("VERTICAL-Corrente motorR",motorR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("VERTICAL-Corrente motorL",motorL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("VERTICAL- chegou na posição alvo e estabilizou", chegouNoTarget());
            telemetry.addData("VERTICAL- getPower", motorR.getPower());
            telemetry.addData("VERTICAL- getPower", motorR.getPower());
            telemetry.addData("VERTICAL- isBusy", isBusy);
            //telemetry.addData("",);

        }
    }


}
