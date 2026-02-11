# Murchison House Trajectory Collection Checklist

Use this checklist when validating indoor map and floor-switch behavior in production data collection.

## 1. Coverage Passes

- Walk all publicly accessible corridors/areas in Murchison House.
- Cover each corridor/area at least 2 times.
- Keep phone visible (not in backpack), screen on, and recording continuous per run.

## 2. Outside-to-Inside Transitions

- Record at least one trajectory that starts outside and walks into the ground floor.
- Record at least one trajectory that starts outside and reaches another floor (not only ground floor).

## 3. Vertical Movement (Stairs and Elevators)

- Record all stairs and all elevators used in the building.
- For each elevation change, include at least 20 m walking before the change and 20 m after.
- Add a marker near each staircase/elevator transition if possible.

## 4. Indoor Map/Floor Feature Validation

- Confirm nearby venue outlines appear automatically on the map.
- Tap a venue and verify:
  - the selected venue name is shown,
  - the indoor floorplan overlay is displayed,
  - floor switching works with:
    - floor up/down buttons,
    - floor dropdown,
    - auto-floor mode.
- During recording, verify uploaded trajectory uses the selected venue tag.

## 5. Suggested Per-Run Metadata to Track

- Date/time, tester name, phone model.
- Start point and end point.
- Floors covered.
- Whether stairs/elevators were included.
- Any GNSS/Wi-Fi/BLE issues.
