#include "gldisplay.h"
#include <QtOpenGL/QtOpenGL>
#include <QtWidgets>

//default draw function (drawing pyramid shape)
void static draw()
{
	glBegin(GL_QUADS);
	glNormal3f(0, 0, -1);
	glVertex3f(-1, -1, 0);
	glVertex3f(-1, 1, 0);
	glVertex3f(1, 1, 0);
	glVertex3f(1, -1, 0);
	glEnd();

	glBegin(GL_TRIANGLES);
	glNormal3f(0, -1, 0.707);
	glVertex3f(-1, -1, 0);
	glVertex3f(1, -1, 0);
	glVertex3f(0, 0, 1.2);
	glEnd();

	glBegin(GL_TRIANGLES);
	glNormal3f(1, 0, 0.707);
	glVertex3f(1, -1, 0);
	glVertex3f(1, 1, 0);
	glVertex3f(0, 0, 1.2);
	glEnd();

	glBegin(GL_TRIANGLES);
	glNormal3f(0, 1, 0.707);
	glVertex3f(1, 1, 0);
	glVertex3f(-1, 1, 0);
	glVertex3f(0, 0, 1.2);
	glEnd();

	glBegin(GL_TRIANGLES);
	glNormal3f(-1, 0, 0.707);
	glVertex3f(-1, 1, 0);
	glVertex3f(-1, -1, 0);
	glVertex3f(0, 0, 1.2);
	glEnd();
}

GLDisplay::GLDisplay(QWidget *parent)
	: QGLWidget(parent)
{
	xRot = 0;
	yRot = 0;
	zRot = 0;
	scale = 1.0;
	//drawFunction = draw;
}

GLDisplay::~GLDisplay()
{
}

QSize GLDisplay::minimumSizeHint() const
{
	return QSize(50, 50);
}

QSize GLDisplay::sizeHint() const
{
	return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360 * 16)
		angle -= 360 * 16;
}

void GLDisplay::setXRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != xRot)
	{
		xRot = angle;
		emit xRotationChanged(angle);
		updateGL();
	}
}

void GLDisplay::setYRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != yRot)
	{
		yRot = angle;
		emit yRotationChanged(angle);
		updateGL();
	}
}
void GLDisplay::setZRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != zRot)
	{
		zRot = angle;
		emit zRotationChanged(angle);
		updateGL();
	}
}

void GLDisplay::initializeGL()
{
	qglClearColor(Qt::black);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	static GLfloat lightPosition[4] = { 0, 0, 10, 1.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void GLDisplay::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -10.0);
	glRotated(xRot, 1.0, 0.0, 0.0);
	glRotatef(yRot, 0.0, 1.0, 0.0);
	glRotatef(zRot, 0.0, 0.0, 1.0);
	glScalef(scale, scale, scale);
	char pixels[] = {
		255, 0, 0, 0,
		0, 255, 0, 0
	};
	drawImage(1, 2, pixels);
}

void GLDisplay::resizeGL(int width, int height)
{
	int side = qMin(width, height);
	glViewport((width - side) / 2, (height - side) / 2, side, side);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
#ifdef QT_OPENGL_ES_1
	glOrthof(-2, +2, -2, +2, 1.0, 15.0);
#else
	glOrtho(-2, +2, -2, +2, 1.0, 15.0);
#endif
	glMatrixMode(GL_MODELVIEW);
}

void GLDisplay::mousePressEvent(QMouseEvent *event)
{
	lastPos = event->pos();
}

void GLDisplay::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();
	int dy = event->y() - lastPos.y();

	if (event->buttons() & Qt::LeftButton)
	{
		setXRotation(xRot + dy);
		setYRotation(yRot + dx);
	}
	else if (event->buttons() & Qt::RightButton)
	{
		setXRotation(xRot + dy);
		setZRotation(zRot + dx);
	}

	lastPos = event->pos();
}

void GLDisplay::wheelEvent(QWheelEvent *event)
{
	QPoint numDegrees = event->angleDelta() / 8;

	if (!numDegrees.isNull()) {
		QPoint numSteps = numDegrees / 15;
		setScale(scale + numSteps.y() / 10.0);
	}
	event->accept();
}

void GLDisplay::setScale(double scale)
{
	this->scale = scale;
	if (this->scale < 0.1)
		this->scale = 0.1;
	else if (this->scale > 2.0)
		this->scale = 2.0;
	updateGL();
}

void GLDisplay::drawImage(GLsizei width, GLsizei height, GLvoid *data)
{
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	// Display the OpenGL texture map
	glColor4f(1, 1, 1, 1);

	glBegin(GL_QUADS);

	// upper left
	glTexCoord2f(-1.0, -1.0);
	glVertex2f(-1.0, -1.0);
	// upper right
	glTexCoord2f(1.0, -1.0);
	glVertex2f(1.0, -1.0);
	// bottom right
	glTexCoord2f(1.0, 1.0);
	glVertex2f(1.0, 1.0);
	// bottom left
	glTexCoord2f(-1.0, 1.0);
	glVertex2f(-1.0, 1.0);

	glEnd();

	//updateGL();

}