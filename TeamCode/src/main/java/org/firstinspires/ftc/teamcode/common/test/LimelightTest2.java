package org.firstinspires.ftc.teamcode.common.test;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.List;


@TeleOp (name = "Limelight Teste")
public class LimelightTest2 extends OpMode {
    Limelight3A limelight;
    LLResultTypes.FiducialResult fiducialResult;
    Pose3D botPose3d;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, HardwareNames.limelight);
        limelight.setPollRateHz(100); // Isso define quantas vezes pedimos dados ao Limelight (100 vezes por segundo)
        limelight.start(); // Isso diz ao Limelight para começar a procurar!


    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {

        limelight.pipelineSwitch(0); // Muda para o pipeline número 0
        LLResult result = limelight.getLatestResult();
           /*
        if (result != null && result.isValid()) {





                telemetry.update();
            }
        }
        */

        LLResult result1 = limelight.getLatestResult();
        if (result1 != null) {
            if (result1.isValid()) {
               botPose3d = result1.getBotpose_MT2();
                // Use os dados de botpose
            }
        }

        telemetry.addData("posição 3d ", botPose3d);
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // Quão longe o alvo está para a esquerda ou direita (graus)
            double ty = result.getTy(); // Quão longe o alvo está para cima ou para baixo (graus)
            double ta = result.getTa(); // Quão grande o alvo parece (0%-100% da imagem)
            result.getPipelineIndex();
            telemetry.addData("Alvo X", tx);
            telemetry.addData("Alvo Y", ty);
            telemetry.addData("Área do Alvo", ta);

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (!fiducials.isEmpty()) {
            LLResultTypes.FiducialResult fiducial = fiducials.get(0);
            Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();

            // Obter a posição (translação)
            double x = robotPoseTargetSpace.getPosition().x;
            double y = robotPoseTargetSpace.getPosition().y;
            double z = robotPoseTargetSpace.getPosition().z;
            // Obter a rotação
            double roll = robotPoseTargetSpace.getOrientation().getRoll();
            double pitch = robotPoseTargetSpace.getOrientation().getPitch();
            double yaw = robotPoseTargetSpace.getOrientation().getYaw();
            telemetry.addData("Posição do robô relativa ao AprilTag",
                    String.format("X=%.2f, Y=%.2f, Z=%.2f", x, y, z));
            telemetry.addData("Rotação do robô relativa ao AprilTag",
                    String.format("Roll=%.2f, Pitch=%.2f, Yaw=%.2f", roll, pitch, yaw));
        }
        else {
            telemetry.addData("Limelight", "Sem Alvos");
        }

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // O numerous ID do fiducial
            LLResultTypes.FiducialResult detection = null;
           // double x = detection.getTargetXDegrees(); // Onde está (esquerda-direita)
           // double y = detection.getTargetYDegrees(); // Onde está (cima-baixo)
            YawPitchRollAngles StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getOrientation();
        }

        }
        if(gamepad1.a){
            limelight.captureSnapshot("auto_pov_10s");
        }
    }
}
