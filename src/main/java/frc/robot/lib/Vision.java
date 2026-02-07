package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

  public double getXVisionSet() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("6083_Vision");
    // 注意：這裡縮排是 4 格 (2+2)
    return table.getDoubleTopic("closest_x").subscribe(0.0).get();
  } // 這裡縮排是 2 格 (與方法開頭對齊)

  public double getYVisionSet() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("6083_Vision");
    return table.getDoubleTopic("closest_y").subscribe(0.0).get();
  }
}