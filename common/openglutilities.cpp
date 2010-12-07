#include "openglutilities.h"

void OpenGlUtilities::drawSphere(const QVector3D &pos, const float radius, const int subdivisions, const QColor color)
{
    GLUquadricObj *quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);

    glPushMatrix();
    glTranslatef(pos.x(), pos.y(), pos.z());
    glColor4f(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    gluSphere(quadric, radius, subdivisions, subdivisions);
    glPopMatrix();

    gluDeleteQuadric(quadric);
}

void OpenGlUtilities::drawSphere(const QVector3D &pos)
{
    OpenGlUtilities::drawSphere(pos, 2, 5, QColor(255,255,0));
}
