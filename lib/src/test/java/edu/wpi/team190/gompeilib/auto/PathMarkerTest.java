package edu.wpi.team190.gompeilib.auto;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class PathMarkerTest {
  @Test
  public void testIsAZoneWithAnEnd() {
    assertTrue(new PathMarker("INTAKE", 0.6, 2.1).isZone());
  }

  @Test
  public void testIsAPointWithoutAnEnd() {
    assertFalse(new PathMarker("STOP_INTAKE", 2.1, -1.0).isZone());
  }

  @Test
  public void testRequiresACommand() {
    assertThrows(NullPointerException.class, () -> new PathMarker(null, 0.0, -1.0));
  }
}
