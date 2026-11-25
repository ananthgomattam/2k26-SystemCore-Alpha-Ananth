package edu.wpi.team190.gompeilib.auto;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

public class AutoPathTest extends GompeiLibTestBase {
  private static final String PATH =
      """
      {
        "version": "2025.0",
        "waypoints": [
          {
            "anchor": {"x": 2.0, "y": 2.0},
            "prevControl": null,
            "nextControl": {"x": 3.0, "y": 2.0},
            "isLocked": false,
            "linkedName": null
          },
          {
            "anchor": {"x": 5.0, "y": 2.0},
            "prevControl": {"x": 4.0, "y": 2.0},
            "nextControl": null,
            "isLocked": false,
            "linkedName": null
          }
        ],
        "rotationTargets": [],
        "constraintZones": [],
        "pointTowardsZones": [],
        "eventMarkers": [MARKERS],
        "globalConstraints": {
          "maxVelocity": 2.0,
          "maxAcceleration": 2.0,
          "maxAngularVelocity": 100.0,
          "maxAngularAcceleration": 200.0,
          "nominalVoltage": 12.0,
          "unlimited": false
        },
        "goalEndState": {"velocity": 0, "rotation": 0.0},
        "reversed": false,
        "folder": null,
        "idealStartingState": {"velocity": 0, "rotation": 0.0},
        "useDefaultConstraints": false
      }
      """;

  private static final String INTAKE_ZONE =
      """
      {"name": "INTAKE", "waypointRelativePos": 0.25, "endWaypointRelativePos": 0.5,
       "command": {"type": "named", "data": {"name": "INTAKE"}}}
      """;

  private static final String STOP_INTAKE_POINT =
      """
      {"name": "STOP_INTAKE", "waypointRelativePos": 0.75, "endWaypointRelativePos": null,
       "command": {"type": "named", "data": {"name": "STOP_INTAKE"}}}
      """;

  @TempDir Path folder;

  private AutoPath load(String... markers) throws Exception {
    Path file = folder.resolve("TEST.path");
    Files.writeString(file, PATH.replace("MARKERS", String.join(",", markers)));
    return AutoPath.load(file.toFile());
  }

  @Test
  public void testReadsZonesAndPoints() throws Exception {
    AutoPath path = load(INTAKE_ZONE, STOP_INTAKE_POINT);
    assertEquals(
        List.of(new PathMarker("INTAKE", 0.25, 0.5), new PathMarker("STOP_INTAKE", 0.75, -1.0)),
        path.markers());
  }

  @Test
  public void testGivesPathPlannerThePathWithoutMarkers() throws Exception {
    AutoPath path = load(INTAKE_ZONE, STOP_INTAKE_POINT);
    assertTrue(path.path().getEventMarkers().isEmpty());
    assertEquals(2, path.path().getWaypoints().size());
    assertEquals(5.0, path.path().getWaypoints().get(1).anchor().getX(), 1e-9);
  }

  @Test
  public void testLoadsAPathWithNoMarkers() throws Exception {
    AutoPath path = load();
    assertTrue(path.markers().isEmpty());
    assertEquals(2, path.path().getWaypoints().size());
  }

  @Test
  public void testRunsTheNamedCommandEvenWhenTheMarkerIsNamedDifferently() throws Exception {
    AutoPath path =
        load(
            """
            {"name": "Intake Zone", "waypointRelativePos": 0.25, "endWaypointRelativePos": 0.5,
             "command": {"type": "named", "data": {"name": "INTAKE"}}}
            """);
    assertEquals("INTAKE", path.markers().get(0).command());
  }

  @Test
  public void testUsesTheMarkerNameWithoutACommand() throws Exception {
    AutoPath path =
        load(
            """
            {"name": "SHOOT", "waypointRelativePos": 0.5, "endWaypointRelativePos": null,
             "command": null}
            """);
    assertEquals(new PathMarker("SHOOT", 0.5, -1.0), path.markers().get(0));
  }

  @Test
  public void testMirrorKeepsTheMarkers() throws Exception {
    AutoPath path = load(INTAKE_ZONE, STOP_INTAKE_POINT);
    AutoPath mirrored = path.mirror();
    assertEquals(path.markers(), mirrored.markers());
    assertNotEquals(
        path.path().getWaypoints().get(0).anchor().getY(),
        mirrored.path().getWaypoints().get(0).anchor().getY(),
        1e-9);
  }
}
