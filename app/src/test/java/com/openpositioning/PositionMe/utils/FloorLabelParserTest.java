package com.openpositioning.PositionMe.utils;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class FloorLabelParserTest {

    @Test
    public void parseLogicalFloorLabel_recognizesGroundFloorVariants() {
        assertEquals(Integer.valueOf(0), FloorLabelParser.parseLogicalFloorLabel("GF"));
        assertEquals(Integer.valueOf(0), FloorLabelParser.parseLogicalFloorLabel("Ground"));
    }

    @Test
    public void parseLogicalFloorLabel_recognizesNumberWithTrailingF() {
        assertEquals(Integer.valueOf(1), FloorLabelParser.parseLogicalFloorLabel("1F"));
        assertEquals(Integer.valueOf(2), FloorLabelParser.parseLogicalFloorLabel("2F"));
    }

    @Test
    public void parseLogicalFloorLabel_recognizesNumberWithLeadingF() {
        assertEquals(Integer.valueOf(1), FloorLabelParser.parseLogicalFloorLabel("F1"));
        assertEquals(Integer.valueOf(2), FloorLabelParser.parseLogicalFloorLabel("F2"));
    }

    @Test
    public void parseLogicalFloorLabel_recognizesBasementAndLevelWithTrailingF() {
        assertEquals(Integer.valueOf(-1), FloorLabelParser.parseLogicalFloorLabel("B1F"));
        assertEquals(Integer.valueOf(2), FloorLabelParser.parseLogicalFloorLabel("L2F"));
    }
}
