package org.frogforce503.lib.util;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * A simple utility class managing Shuffleboard layouts and components
 */
public class ShuffleboardUtil {

    private NetworkTable networkTable;
    private ShuffleboardLayout sLayout;
    private ShuffleboardComponent sComponent;

    /**
     * Use to create new ShuffleborardUtil instance
     * (Make sure to use ONE instance per class)
     * 
     * @return New ShuffleboardUtil Instance
     */
    public static ShuffleboardUtil getNewSUtil() {
        return new ShuffleboardUtil();
    }

    // /**
    // * Use to set layout outputs (Shortcut instead of caling getLayout())
    // *
    // * @apiNote MUST INITIALIZE LAYOUT AND THEN USE METHOD, OTHERWISE NULL
    // EXCEPTION
    // * THROWED
    // */
    // public ShuffleboardLayout get_sLayout() {
    // return sLayout;
    // }

    // /**
    // * Use to set component outputs (Shortcut instead of caling getComponent())
    // *
    // * @apiNote MUST INITIALIZE COMPONENT AND THEN USE METHOD, OTHERWISE NULL
    // * EXCEPTION
    // * THROWED
    // */
    // public ShuffleboardComponent get_sComponent() {
    // return sComponent;
    // }

    /**
     * Use this method to initialize a Shuffleboard layout.
     * For subsystems, use this in the subsystem constructor (if subsystem values
     * wanted to be put to Shuffleboard)
     * 
     * @param name    Layout Name
     * @param tabName Specified tab of the layout
     * @param type    Type of layout (BuiltInLayouts)
     */
    public ShuffleboardLayout getLayout(String name, String tabName, BuiltInLayouts type) {
        networkTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName)
                .getSubTable(name);
        sLayout = Shuffleboard.getTab(tabName).getLayout(name, type);
        return sLayout;
    }

    /**
     * Use this method to intialize a Shuffleboard component.
     * For subsystems, use this in the subsystem constructor (if subsystem values
     * wanted to be put to Shuffleboard)
     * 
     * @param name    Component Name
     * @param tabName Specified tab of the component
     * @param type    Type of component (BuiltInWidgets)
     */
    public ShuffleboardComponent getComponent(String name, String tabName, BuiltInWidgets type) {
        networkTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName)
                .getSubTable(name);
        sComponent = Shuffleboard.getTab(tabName).getComponents()
                .get(Shuffleboard.getTab(tabName).getComponents().indexOf(name));
        return sComponent;
    }

    /**
     * Use this method to set the properties of a Shuffleboard layout.
     * For subsystems, use this in initTelemetry() (if subsystem values wanted to be
     * put to Shuffleboard)
     * 
     * @param column     Specified column of the layout
     * @param row        Specified row of the layout
     * @param width      Specified width of the layout
     * @param height     Specified height of the layout
     * @param properties Specified properties of the layout
     */
    public ShuffleboardLayout setLayout(int column, int row, int width, int height, Map<String, Object> properties) {
        sLayout.withPosition(column, row).withSize(width, height).withProperties(properties);
        return sLayout;
    }

    /**
     * Use this method to set the properties of a Shuffleboard component.
     * For subsystems, use this in initTelemetry() (if subsystem values wanted to be
     * put to Shuffleboard)
     * 
     * @param column     Specified column of the component
     * @param row        Specified row of the component
     * @param width      Specified width of the component
     * @param height     Specified height of the component
     * @param properties Specified properties of the component
     */
    public ShuffleboardComponent setComponent(int column, int row, int width, int height,
            Map<String, Object> properties) {
        sComponent.withPosition(column, row).withSize(width, height).withProperties(properties);
        return sComponent;
    }

    /**
     * Use this method to output certain values to NetworkTables
     * For subsystems, use this in outputTelemetry() (if subsystem values wanted to
     * be put to Shuffleboard)
     * 
     * @param entryAndOutput Key: NetworkTables entry, Value: value wanted to output
     *                       to NetworkTables
     */
    public void outputToNetworkTables(Map<String, Object> entryAndOutput) {
        for (Map.Entry<String, Object> pair : entryAndOutput.entrySet()) {

            NetworkTableEntry nE = networkTable.getEntry(pair.getKey());
            Object pV = pair.getValue();

            if (pV instanceof Boolean) {
                nE.setBoolean((Boolean) pV);
            } else if (pV instanceof boolean[]) {
                nE.setBooleanArray((boolean[]) pV);
            } else if (pV instanceof Double) {
                nE.setDouble((Double) pV);
            } else if (pV instanceof double[]) {
                nE.setDoubleArray((double[]) pV);
            } else if (pV instanceof byte[]) {
                nE.setRaw((byte[]) pV);
            } else if (pV instanceof String) {
                nE.setString((String) pV);
            } else if (pV instanceof String[]) {
                nE.setStringArray((String[]) pV);
            } else {
                nE.setValue(pV);
            }

        }
    }

    /**
     * Constructor: Initialize with reference
     * InitTelemetry: Size & Output
     * OutputTelemetry: NetworktTables setting & output
     * 
     * NetworkTable organization in constructor
     * could be sLayout or sComponent
     * 
     * Write separate subclasses in ShuffleboardUtil
     */
}