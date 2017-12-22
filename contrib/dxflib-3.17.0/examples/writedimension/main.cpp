	#include "dl_dxf.h"
 
int main() {
    DL_Dxf dxf;
    DL_WriterA* dw = dxf.out("dimension.dxf", DL_Codes::AC1015);
 
    // section header:
    dxf.writeHeader(*dw);
    dw->sectionEnd();
 
    // section tables:
    dw->sectionTables();

    // VPORT:
    dxf.writeVPort(*dw);

    // LTYPE:
    dw->tableLinetypes(1);
    dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
    dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
    dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
    dw->tableEnd();

    // LAYER:
    dw->tableLayers(1);
    dxf.writeLayer(
        *dw,
        DL_LayerData("0", 0),
        DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS")
    );
    dw->tableEnd();

    // STYLE:
    dw->tableStyle(1);
    DL_StyleData style("Standard", 0, 0.0, 1.0, 0.0, 0, 2.5, "txt", "");
    style.bold = false;
    style.italic = false;
    dxf.writeStyle(*dw, style);
    dw->tableEnd();

    // VIEW:
    dxf.writeView(*dw);

    // UCS:
    dxf.writeUcs(*dw);

    // APPID:
    dw->tableAppid(1);
    dxf.writeAppid(*dw, "ACAD");
    dw->tableEnd();

    // DIMSTYLE:
    dxf.writeDimStyle(*dw, 2.5, 0.625, 0.625, 0.625, 2.5);

    // BLOCK_RECORD:
    dxf.writeBlockRecord(*dw);
    dw->tableEnd();

    dw->sectionEnd();
 
    // BLOCK:
    dw->sectionBlocks();
    dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Model_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space0");
    dw->sectionEnd();
 
    // ENTITIES:
    dw->sectionEntities();
 
    DL_Attributes attributes("0", 256, -1, -1, "BYLAYER");

    // LINE:
    DL_LineData lineData(10, 5, 0, 30, 5, 0);
    dxf.writeLine(*dw, lineData, attributes);
 
    // DIMENSION:
    DL_DimensionData dimData(10.0,                  // def point (dimension line pos)
                             50.0,
                             0.0,
                             0,                     // text pos (irrelevant if flag 0x80 (128) set for type
                             0,
                             0.0,
                             0x1,                   // type: aligned with auto text pos (0x80 NOT set)
                             8,                     // attachment point: bottom center
                             2,                     // line spacing: exact
                             1.0,                   // line spacing factor
                             "",                    // text
                             "Standard",            // style
                             0.0,                   // text angle
                             1.0,                   // linear factor
                             1.0);                  // dim scale

    DL_DimAlignedData dimAlignedData(10.0,          // extension point 1
                                     5.0,
                                     0.0,
                                     30.0,          // extension point 2
                                     5.0,
                                     0.0);

    dxf.writeDimAligned(*dw, dimData, dimAlignedData, attributes);
 
    // end section ENTITIES:
    dw->sectionEnd();
    dxf.writeObjects(*dw, "MY_OBJECTS");
    dxf.writeObjectsEnd(*dw);
 
    dw->dxfEOF();
    dw->close();
    delete dw;
 
    return 0;
}
