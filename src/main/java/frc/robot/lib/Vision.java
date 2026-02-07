package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleSubscriber;

public class Vision {
  private final DoubleSubscriber closestXSub;
  private final DoubleSubscriber closestYSub;

  public Vision() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("6083_Vision");
    closestXSub = table
        .getDoubleTopic("closest_x")
        .subscribe(0.0);
    closestYSub = table
        .getDoubleTopic("closest_y")
        .subscribe(0.0);
  }

  public double getXVisionSet() {
    return closestXSub.get();
  }

  public double getYVisionSet() {
    return closestYSub.get();
  }
}
