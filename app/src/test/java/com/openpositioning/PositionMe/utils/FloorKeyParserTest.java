package com.openpositioning.PositionMe.utils;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class FloorKeyParserTest {

    @Test
    public void parseFloorNameSupportsGroundVariants() {
        assertEquals(0, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("GF"));
        assertEquals(0, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("Ground Floor"));
        assertEquals(-1, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("LG"));
        assertEquals(-1, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("Lower Ground"));
        assertEquals(1, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("UG"));
        assertEquals(1, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("Upper Ground"));
        assertEquals(-2, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("B2"));
        assertEquals(-1, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("-1"));
        assertEquals(3, FloorKeyParser.parseFloorNameToAbsoluteFloorOrZero("L3"));
    }

    @Test
    public void parseRawFloorValueSupportsSemanticStrings() {
        assertEquals(Integer.valueOf(-1), FloorKeyParser.parseRawFloorValue("LG"));
        assertEquals(Integer.valueOf(1), FloorKeyParser.parseRawFloorValue("UGF"));
        assertEquals(Integer.valueOf(2), FloorKeyParser.parseRawFloorValue(2));
    }

    @Test
    public void parseRawFloorValueReturnsNullForUnknownText() {
        assertNull(FloorKeyParser.parseRawFloorValue("unknown"));
    }
}
