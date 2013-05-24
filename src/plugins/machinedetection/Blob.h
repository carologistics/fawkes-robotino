/*
 * Blob.h
 *
 *  Created on: 08.02.2013
 *      Author: daniel
 */

#ifndef BLOB_H_
#define BLOB_H_

#include <string>

class Blob {
public:
	Blob(unsigned char* buffer, int buffer_width, int buffer_height, int threshold, int x,
			int y);
	virtual ~Blob();
	void grow();
	int getCenterX();
	int getCenterY();
	int getLeftX() const;
	int getLowerY() const;
	int getRightX() const;
	int getUpperY() const;
	int getDiameter() const;
	void setLeftX(int leftX);
	void setLowerY(int lowerY);
	void setRightX(int rightX);
	void setUpperY(int upperY);

private:
	unsigned char* buffer_;
	int buffer_width_;
	int buffer_height_;
	int threshold_;
	int center_x_;
	int center_y_;
	int left_x_;
	int right_x_;
	int upper_y_;
	int lower_y_;
	void grow_left();
	void grow_right();
	void grow_up();
	void grow_down();


};

#endif /* BLOB_H_ */
