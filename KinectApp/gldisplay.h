#ifndef GLDISPLAY_H
#define GLDISPLAY_H

#include <QGLWidget>

class GLDisplay : public QGLWidget
{
	Q_OBJECT

public:
	GLDisplay(QWidget *parent);
	~GLDisplay();

	void drawImage();
	void drawShape();
	void drawSphere(float x, float y, float z, float size);
	void drawPointCloud();
	void setDrawingColor(float r, float g, float b);
	void setDrawingColor(QColor color);

	void setImage(GLsizei width, GLsizei height, const GLvoid *data);
	void setPointCloudData(GLsizei width, GLsizei height, const float *data);
	void setPoints(int size, const float *data);

public slots:
	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);
	void setScale(double scale);
	void update();
	void resetRotation();

signals:
	void xRotationChanged(int angle);
	void yRotationChanged(int angle);
	void zRotationChanged(int angle);

protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int width, int height);

	QSize minimumSizeHint() const;
	QSize sizeHint() const;

	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
private:
	void drawPoints();

	double scale;
	bool lockRotation;
	int xRot;
	int yRot;
	int zRot;

	QPoint lastPos;

	const float *points;
	int pointsSize;
	const float *pointsCloudData;
	const GLvoid *texData;
	GLsizei texWidth;
	GLsizei texHeight;
	
};

#endif // GLDISPLAY_H
