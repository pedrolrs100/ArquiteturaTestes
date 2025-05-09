package org.firstinspires.ftc.teamcode.robot.Sensors.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.List;

public class Limelight {
    private final Limelight3A limelight;
    private LLResultTypes.FiducialResult fiducialResult;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, HardwareNames.limelight);
        limelight.setPollRateHz(100); // Isso define quantas vezes pedimos dados ao Limelight (100 vezes por segundo)
        limelight.start(); // Isso diz ao Limelight para começar a procurar!
        limelight.pipelineSwitch(0); // Muda para o pipeline número 0

    }

    public double getZdistanceFromAprilTag() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (!fiducials.isEmpty()) {
            LLResultTypes.FiducialResult fiducial = fiducials.get(0);
            Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();

            // Obter a posição (translação)
            return robotPoseTargetSpace.getPosition().z;

        }
        return 0;
    }

    public void desativar() {
        limelight.shutdown();
    }
}
