package frc.robot.lib;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Vision {
  private final DoubleSubscriber closestSubX;
  private final DoubleSubscriber closestSubY;

  public Vision() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("6083_Vision");
    closestSubX = table
        .getDoubleTopic("closest_x")
        .subscribe(0.0);
    closestSubY = table
        .getDoubleTopic("closest_y")
        .subscribe(0.0);
  }

  public double getVisionX() {
    return closestSubX.get();
  }

  public double getVisionY() {
    return closestSubY.get();
  }
}
