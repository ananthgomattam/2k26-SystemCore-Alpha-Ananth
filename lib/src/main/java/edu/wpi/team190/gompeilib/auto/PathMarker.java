package edu.wpi.team190.gompeilib.auto;

import java.util.Objects;

public record PathMarker(String command, double start, double end) {

  public PathMarker {
    Objects.requireNonNull(command, "command");
  }

  public boolean isZone() {
    return end >= 0;
  }
}
