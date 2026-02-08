package frc.robot.lib;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
  private final DoubleSubscriber closestSubX;
  private final DoubleSubscriber closestSubY;
  private final IntegerArraySubscriber allIdsSub;
  private final DoubleArraySubscriber allXsSub;
  private final DoubleArraySubscriber allYsSub;

  public Vision() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("6083_Vision");
    closestSubX = table
        .getDoubleTopic("closest_x")
        .subscribe(0.0);
    closestSubY = table
        .getDoubleTopic("closest_dist") // python code uses "closest_dist" for y value, so we will do the same here
        .subscribe(0.0);
    allIdsSub = table
        .getIntegerArrayTopic("all_ids")
        .subscribe(new long[] {});
    allXsSub = table
        .getDoubleArrayTopic("all_xs")
        .subscribe(new double[] {});
    allYsSub = table
        .getDoubleArrayTopic("all_ys")
        .subscribe(new double[] {});

  }

  public double getVisionX() {
    return closestSubX.get();
  }

  public double getVisionY() {
    return closestSubY.get();
  }

  // 讀取球ID
  public long[] getAllBallIds() {
    return allIdsSub.get();
  }

  public double[] getBallById(int targetId) {
    long[] ids = allIdsSub.get();
    double[] xs = allXsSub.get();
    double[] ys = allYsSub.get();

    // 確保數據同步 (長度要一樣)
    if (ids.length != xs.length || ids.length != ys.length) {
      return null;
    }

    // 搜尋 ID
    for (int i = 0; i < ids.length; i++) {
      if (ids[i] == targetId) {
        return new double[] { xs[i], ys[i] }; // 找到了 回傳座標
      }
    }
    return null; // 沒找到這顆球
  }

  public double[] getNextClosestBall(int currentLockedId) {
    long[] ids = allIdsSub.get();
    double[] xs = allXsSub.get();
    double[] ys = allYsSub.get();

    double maxY = -1.0;
    int bestIndex = -1;

    for (int i = 0; i < ids.length; i++) {
      if (ids[i] == currentLockedId)
        continue; // 跳過目前鎖定的那顆

      // 找第二近的 (Y 越大越近)
      if (ys[i] > maxY) {
        maxY = ys[i];
        bestIndex = i;
      }
    }

    if (bestIndex != -1) {
      return new double[] { xs[bestIndex], ys[bestIndex], (double) ids[bestIndex] };
    }
    return null;
  }
  public double[] getClosestBallWithId() {
    long[] ids = allIdsSub.get();
    double[] xs = allXsSub.get();
    double[] ys = allYsSub.get();

    if (ids.length == 0) return null;

    int bestIndex = -1;
    double maxScore = -1.0;

    for (int i = 0; i < ids.length; i++) {
        if (ys[i] > maxScore) {  // Y 越大越近
            maxScore = ys[i];
            bestIndex = i;
        }
    }

    if (bestIndex != -1) {
        return new double[] { xs[bestIndex], ys[bestIndex], (double)ids[bestIndex] };
    }
    return null;
}
}