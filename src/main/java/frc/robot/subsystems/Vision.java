// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  PhotonCamera camera;
  List<PhotonPipelineResult> pipeline;
  PhotonPipelineResult LatestResult;
  PhotonTrackedTarget LatestBestTarget;
  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("photonvision");
    pipeline = camera.getAllUnreadResults();
    if (pipeline.size() > 0){
      LatestResult = pipeline.get(pipeline.size()-1);
      LatestBestTarget = LatestResult.getBestTarget();
      System.out.println(LatestBestTarget);
      System.out.println("April Tag: " + LatestBestTarget.getFiducialId());

    }else{
      System.out.println("No new updates");
    }
   
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
