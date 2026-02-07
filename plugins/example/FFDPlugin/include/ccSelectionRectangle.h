#pragma once

#include <cc2DViewportObject.h>

class ccSelectionRectangle : public cc2DViewportObject
{
public:
    ccSelectionRectangle();
    ~ccSelectionRectangle() override = default;

    void setRectangle(int x1, int y1, int x2, int y2);
    void setDrawing(bool drawing) { m_isDrawing = drawing; }
    bool isDrawing() const { return m_isDrawing; }

protected:
    void drawMeOnly(CC_DRAW_CONTEXT& context) override;

private:
    int m_x1 = 0;
    int m_y1 = 0;
    int m_x2 = 0;
    int m_y2 = 0;
    bool m_isDrawing = false;
};
