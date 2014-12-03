#include "gldisplay.h"
#include <QtOpenGL/QtOpenGL>
#include <QtWidgets>
#include <gl\GLU.h>

void GLDisplay::drawSphere(float x, float y, float z, float size)
{
	GLUquadricObj* Sphere = gluNewQuadric();
	gluQuadricNormals(Sphere, GLU_SMOOTH);
	//gluQuadricTexture(Sphere, GL_TRUE);

	glTranslatef(x, y, z);
	gluSphere(Sphere, size, 20, 20);
	glTranslatef(-x, -y, -z);
}

void GLDisplay::setDrawingColor(float r, float g, float b)
{
	glColor3f(r, g, b);
}
void GLDisplay::setDrawingColor(QColor color)
{
	int r, g, b;
	color.getRgb(&r, &g, &b);
	glColor3ub(r, g, b);
}

GLDisplay::GLDisplay(QWidget *parent)
: QGLWidget(parent)
{
	xRot = 0;
	yRot = 0;
	zRot = 0;
	scale = 1.0;
	lockRotation = false;
	texData = NULL;
	pointsCloudData = NULL;
	points = NULL;
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(33);
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
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);

	//static GLfloat lightPosition[4] = { 0, 0, 10, 1.0 };
	//glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
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

	if (texData != NULL)
	{
		drawImage();
	}
	//drawSphere(-1.0, 1.0, 0.0, 0.1);
	if (pointsCloudData != NULL)
	{
		drawPointCloud();
	}

	if (points != NULL)
	{
		drawPoints();
	}



}

void GLDisplay::resizeGL(int width, int height)
{
	//glViewport((width - side) / 2, (height - side) / 2, side, side);
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
#ifdef QT_OPENGL_ES_1
	glOrthof(-2, +2, -2, +2, 1.0, 15.0);
#else
	glOrtho(-1, +1, -1, +1, 1.0, 15.0);
#endif
	glMatrixMode(GL_MODELVIEW);
}

void GLDisplay::mousePressEvent(QMouseEvent *event)
{
	if (!lockRotation)
	{
		lastPos = event->pos();
	}
	event->accept();
}

void GLDisplay::mouseMoveEvent(QMouseEvent *event)
{
	if (!lockRotation)
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
	event->accept();
}

void GLDisplay::wheelEvent(QWheelEvent *event)
{
	updateGL();
	if (!lockRotation)
	{
		QPoint numDegrees = event->angleDelta() / 8;
		if (!numDegrees.isNull()) {
			QPoint numSteps = numDegrees / 15;
			setScale(scale + numSteps.y() / 10.0);
		}
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

void GLDisplay::drawImage()
{
	//	lockRotation = true;
	//glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, texData);

	// Display the OpenGL texture map
	glColor4f(1, 1, 1, 1);

	glBegin(GL_QUADS);

	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(-1.0f, -1.0f);

	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(1.0f, -1.0f);

	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(1.0f, 1.0f);

	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(-1.0f, 1.0f);

	glEnd();

}

void GLDisplay::setImage(GLsizei width, GLsizei height, const GLvoid *data)
{
	texWidth = width;
	texHeight = height;
	texData = data;
}

void GLDisplay::setPointCloudData(GLsizei width, GLsizei height, const float *data)
{
	texWidth = width;
	texHeight = height;
	pointsCloudData = data;
	texData = NULL;
}

void GLDisplay::update()
{
	updateGL();
}

void GLDisplay::drawPointCloud()
{
	glDisable(GL_TEXTURE_2D);
	glPointSize(3.0f);
	glBegin(GL_POINTS); //starts drawing of points
	for (int i = 0; i < texWidth * texHeight * 6; i+=6)
	{
		if (*(pointsCloudData + i) == 0.0f)
			continue;
		glColor3f(*(pointsCloudData + i), *(pointsCloudData + i + 1), *(pointsCloudData + i + 2));
		glVertex3f(*(pointsCloudData + i + 3), *(pointsCloudData + i + 4), *(pointsCloudData + i + 5));
	}
	glEnd();//end drawing of points
}

void GLDisplay::resetRotation()
{
	xRot = 0;
	yRot = 0;
	zRot = 0;
	scale = 1.0;
}
void GLDisplay::setPoints(int size, const float *data)
{
	pointsSize = size;
	points = data;
}

void GLDisplay::drawPoints()
{
	for (int i = 0; i < pointsSize * 3; i+=3)
	{
		drawSphere(*(points+i), *(points + i + 1), *(points + i + 2), 0.01f);
	}
}