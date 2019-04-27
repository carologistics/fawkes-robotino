/*
 * Blob.h
 *
 *  Created on: 08.02.2013
 *      Author: daniel
 */

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef BLOB_H_
#define BLOB_H_

#include <string>

class Blob
{
public:
	Blob(unsigned char *buffer, int buffer_width, int buffer_height, int threshold, int x, int y);
	virtual ~Blob();
	void grow();
	int  getCenterX();
	int  getCenterY();
	int  getLeftX() const;
	int  getLowerY() const;
	int  getRightX() const;
	int  getUpperY() const;
	int  getDiameter() const;
	void setLeftX(int leftX);
	void setLowerY(int lowerY);
	void setRightX(int rightX);
	void setUpperY(int upperY);

private:
	unsigned char *buffer_;
	int            buffer_width_;
	int            buffer_height_;
	int            threshold_;
	int            center_x_;
	int            center_y_;
	int            left_x_;
	int            right_x_;
	int            upper_y_;
	int            lower_y_;
	void           grow_left();
	void           grow_right();
	void           grow_up();
	void           grow_down();
};

#endif /* BLOB_H_ */
