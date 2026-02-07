package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    public double getXVisionSet() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("6083_Vision");
        return table.getDoubleTopic("closest_x").subscribe(0.0).get();
    }

    public double getYVisionSet() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("6083_Vision");
        return table.getDoubleTopic("closest_y").subscribe(0.0).get();
    }
}
