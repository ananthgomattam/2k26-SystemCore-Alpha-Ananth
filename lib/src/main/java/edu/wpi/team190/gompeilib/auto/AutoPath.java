package edu.wpi.team190.gompeilib.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import java.io.File;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.wpilib.system.Filesystem;

public record AutoPath(PathPlannerPath path, List<PathMarker> markers) {

  public AutoPath {
    Objects.requireNonNull(path, "path");
    Objects.requireNonNull(markers, "markers");
  }

  public static AutoPath load(String name) throws Exception {
    return load(new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + name + ".path"));
  }

  static AutoPath load(File file) throws Exception {
    String pathFile = Files.readString(file.toPath());
    JSONObject parsed = (JSONObject) new JSONParser().parse(pathFile);

    JSONArray jMarkers = (JSONArray) parsed.get("eventMarkers");
    List<PathMarker> markers = new ArrayList<>();

    for (Object o : jMarkers) {
      JSONObject marker = (JSONObject) o;

      double start = ((Number) marker.get("waypointRelativePos")).doubleValue();

      Object endJson = marker.get("endWaypointRelativePos");
      double end = endJson == null ? -1.0 : ((Number) endJson).doubleValue();

      String command = (String) marker.get("name");
      JSONObject commandJson = (JSONObject) marker.get("command");
      if (commandJson != null && "named".equals(commandJson.get("type"))) {
        command = (String) ((JSONObject) commandJson.get("data")).get("name");
      }

      markers.add(new PathMarker(command, start, end));
    }

    parsed.put("eventMarkers", new JSONArray());

    Method fromJson = PathPlannerPath.class.getDeclaredMethod("fromJson", JSONObject.class);
    fromJson.setAccessible(true);
    PathPlannerPath path = (PathPlannerPath) fromJson.invoke(null, parsed);

    return new AutoPath(path, markers);
  }

  public AutoPath mirror() {
    return new AutoPath(path.mirrorPath(), markers);
  }
}
