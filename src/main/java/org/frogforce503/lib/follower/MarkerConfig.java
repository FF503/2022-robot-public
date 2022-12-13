package org.frogforce503.lib.follower;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MarkerConfig {
    HashMap<String, HashMap<String, Runnable>> markerConfigs = new HashMap<>();
    ArrayList<String> searchZoneNames = new ArrayList<>();
    ArrayList<Path.Marker> searchZones = new ArrayList<>();
    HashMap<String, Supplier<Boolean>> searchZoneEndConditions = new HashMap<>();

    public MarkerConfig addMarker(String name, Runnable onEnter, Runnable insidePeriodic, Runnable onExit) {
        HashMap<String, Runnable> functions = new HashMap<>();
        functions.put("onEnter", onEnter);
        functions.put("insidePeriodic", insidePeriodic);
        functions.put("onExit", onExit);
        markerConfigs.put(name, functions);
        return this;
    }

    public MarkerConfig addMarker(String name, Runnable onEnter) {
        return this.addMarker(name, onEnter, null, null);
    }

    public MarkerConfig addMarker(String name, Runnable onEnter, Runnable onExit) {
        return this.addMarker(name, onEnter, null, onExit);
    }

    public MarkerConfig addSearchZone(String name, Supplier<Boolean> endCondition) {
        this.addMarker("SEARCHZONE" + name, null);
        searchZoneNames.add("SEARCHZONE" + name);
        searchZoneEndConditions.put("SEARCHZONE" + name, endCondition);

        System.out.println("Added " + (endCondition == null ? "" : "Unconventional") + " Search Zone " + name);
        return this;
    }

    public MarkerConfig addSearchZone(String name) {
        return this.addSearchZone(name, null);
    }

    /**
     * To be used only by the Path class when configuring its own markers
     * 
     * @param markers The Path.Marker markers that need to be configured
     */
    public void _configureMarkers(List<Path.Marker> markers) {
        // int searchZoneIndex = 0;
        for (Path.Marker marker : markers) {
            if (!searchZoneNames.contains(marker.name)) {
                HashMap<String, Runnable> functions = markerConfigs.get(marker.name);
                if (functions != null) {
                    Runnable onEnter = functions.get("onEnter");
                    Runnable insidePeriodic = functions.get("insidePeriodic");
                    Runnable onExit = functions.get("onExit");
                    marker.setFunctions(onEnter, insidePeriodic, onExit);
                }
            } else {
                Supplier<Boolean> endCondition = searchZoneEndConditions.get(marker.name);

                if (endCondition != null) {
                    System.out.println("UNCONREGISTERED " + marker.name);
                    marker.stateSZSupplier = endCondition;
                    marker.isUnconventionalSZ = true;
                }

                searchZones.add(marker);
            }
        }
    }

    /**
     * To be used only by the Path class when configuring its own search zones
     * Configure Markers MUST be called before this method.
     * 
     * @param path The path that will be used to feed the CargoFetcher
     */
    public void _registerSearchZones(Path path) {
        int searchZoneCount = 0;
        for (Path.Marker searchZone : searchZones) {
            searchZoneCount++;
        }
        SmartDashboard.putNumber("search zone count", searchZoneCount);
        // path.applySearchZoneWaits(searchZones);
    }
}