#include "ccSelectionRectangle.h"
#include <QOpenGLFunctions_2_1>
#include <algorithm>

#include "FFDDebug.h"

ccSelectionRectangle::ccSelectionRectangle()
    : cc2DViewportObject("Selection Rectangle")
{
    FFD_DEBUG("ccSelectionRectangle CONSTRUCTOR: this=" << this);
    setEnabled(true);
    setVisible(true);
    showNameIn3D(false);
}

ccSelectionRectangle::~ccSelectionRectangle()
{
    FFD_DEBUG("ccSelectionRectangle DESTRUCTOR: this=" << this);
}

void ccSelectionRectangle::setRectangle(int x1, int y1, int x2, int y2)
{
    FFD_DEBUG("ccSelectionRectangle::setRectangle: this=" << this << ", x1=" << x1 << ", y1=" << y1 << ", x2=" << x2 << ", y2=" << y2);
    m_x1 = x1;
    m_y1 = y1;
    m_x2 = x2;
    m_y2 = y2;
}

void ccSelectionRectangle::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    FFD_DEBUG("ccSelectionRectangle::drawMeOnly: this=" << this << ", m_isDrawing=" << m_isDrawing);
    if (!m_isDrawing)
        return;

    // Only draw in 2D foreground pass
    if (!MACRO_Draw2D(context) || !MACRO_Foreground(context))
        return;

    QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (!glFunc)
        return;

    // Save current state
    glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LINE_BIT | GL_ENABLE_BIT);
    
    // Disable depth test for 2D overlay
    glFunc->glDisable(GL_DEPTH_TEST);
    
    // Enable blending for transparency
    glFunc->glEnable(GL_BLEND);
    glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Set up 2D orthographic projection
    int halfW = context.glW / 2;
    int halfH = context.glH / 2;
    
    glFunc->glMatrixMode(GL_PROJECTION);
    glFunc->glPushMatrix();
    glFunc->glLoadIdentity();
    glFunc->glOrtho(0, context.glW, 0, context.glH, -1, 1);
    
    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPushMatrix();
    glFunc->glLoadIdentity();
    
    // Normalize rectangle coordinates
    int minX = std::min(m_x1, m_x2);
    int maxX = std::max(m_x1, m_x2);
    int minY = std::min(m_y1, m_y2);
    int maxY = std::max(m_y1, m_y2);
    
    // Flip Y coordinate (OpenGL vs screen coordinates)
    int glMinY = context.glH - maxY;
    int glMaxY = context.glH - minY;
    
    // Draw filled rectangle with transparency
    glFunc->glColor4f(0.0f, 1.0f, 0.0f, 0.2f); // Green with 20% opacity
    glFunc->glBegin(GL_QUADS);
    glFunc->glVertex2i(minX, glMinY);
    glFunc->glVertex2i(maxX, glMinY);
    glFunc->glVertex2i(maxX, glMaxY);
    glFunc->glVertex2i(minX, glMaxY);
    glFunc->glEnd();
    
    // Draw rectangle border
    glFunc->glLineWidth(2.0f);
    glFunc->glColor4f(0.0f, 1.0f, 0.0f, 1.0f); // Solid green
    glFunc->glBegin(GL_LINE_LOOP);
    glFunc->glVertex2i(minX, glMinY);
    glFunc->glVertex2i(maxX, glMinY);
    glFunc->glVertex2i(maxX, glMaxY);
    glFunc->glVertex2i(minX, glMaxY);
    glFunc->glEnd();
    
    // Restore matrices
    glFunc->glMatrixMode(GL_PROJECTION);
    glFunc->glPopMatrix();
    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPopMatrix();
    
    // Restore state
    glFunc->glPopAttrib();
}
