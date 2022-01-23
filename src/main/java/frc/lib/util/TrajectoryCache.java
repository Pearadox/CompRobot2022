package frc.lib.util;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

// Taken from Team 3103, Iron Plaid
public class TrajectoryCache {
  private static HashMap<String, Trajectory> cache = new HashMap<String, Trajectory>();

  public static void add(String key, String jsonPath) {
    try {
      Path trajectoryPath =
          Filesystem.getDeployDirectory().toPath().resolve(jsonPath + ".wpilib.json");
      cache.put(key, TrajectoryUtil.fromPathweaverJson(trajectoryPath));
    } catch (IOException ex) {
      DriverStation.reportError(
          "Unable to open trajectory: " + jsonPath + ".wpilib.json", ex.getStackTrace());
    }
  }

  public static Trajectory get(String key) {
    return cache.get(key);
  }

  public static void clear() {
    cache.clear();
  }
}
