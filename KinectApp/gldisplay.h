#ifndef GLDISPLAY_H
#define GLDISPLAY_H

#include <QGLWidget>

class GLDisplay : public QGLWidget
{
	Q_OBJECT

public:
	GLDisplay(QWidget *parent);
	~GLDisplay();

	void drawImage(GLsizei width, GLsizei height, GLvoid *data);
	void drawShape();

public slots:
	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);
	void setScale(double scale);

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

	double scale;

	int xRot;
	int yRot;
	int zRot;

	QPoint lastPos;
	
};

#endif // GLDISPLAY_H
