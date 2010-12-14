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

void OpenGlUtilities::drawPoint(const QVector3D &pos)
{
//    glColor4f(1.0, 1.0, 0.0, 1.0);
//    glBegin(GL_POINTS);
    glVertex3f(pos.x(), pos.y(), pos.z());
//    glEnd();
}



void OpenGlUtilities::drawAabb(const QVector3D &min, const QVector3D &max, const QVector3D &color)
{
    glLineWidth(1);
    glColor4f(color.x(), color.y(), color.z(), 0.5);

    glBegin(GL_LINE_STRIP);
    glVertex3f(min.x(), min.y(), min.z());
    glVertex3f(max.x(), min.y(), min.z());
    glVertex3f(max.x(), max.y(), min.z());
    glVertex3f(min.x(), max.y(), min.z());
    glVertex3f(min.x(), min.y(), min.z());
    glVertex3f(min.x(), min.y(), max.z());
    glVertex3f(max.x(), min.y(), max.z());
    glVertex3f(max.x(), max.y(), max.z());
    glVertex3f(min.x(), max.y(), max.z());
    glVertex3f(min.x(), min.y(), max.z());
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(min.x(), max.y(), min.z());
    glVertex3f(min.x(), max.y(), max.z());
    glVertex3f(max.x(), min.y(), min.z());
    glVertex3f(max.x(), min.y(), max.z());
    glVertex3f(max.x(), max.y(), min.z());
    glVertex3f(max.x(), max.y(), max.z());
    glEnd();
}
