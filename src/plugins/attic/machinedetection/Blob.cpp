/*
 * Blob.cpp
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

#include "Blob.h"

#include <algorithm>

Blob::Blob(unsigned char *buffer, int buffer_width, int buffer_height, int threshold, int x, int y)
: buffer_(buffer),
  buffer_width_(buffer_width),
  buffer_height_(buffer_height),
  threshold_(threshold),
  center_x_(x),
  center_y_(y)
{
	lower_y_ = center_y_;
	upper_y_ = center_y_;

	left_x_  = center_x_;
	right_x_ = center_x_;
}

Blob::~Blob()
{
	// TODO Auto-generated destructor stub
}

int
Blob::getDiameter() const
{
	return (std::min(right_x_ - left_x_, lower_y_ - upper_y_));
}

/**
 * expands the blob in all directions.
 */
void
Blob::grow()
{
	grow_left();
	grow_right();
	grow_up();
	grow_down();
}

void
Blob::grow_left()
{
	while (buffer_[center_y_ * buffer_width_ + left_x_] == 255) {
		if (left_x_ > 0) {
			left_x_--;
		} else {
			return;
		}
		return;
	}
}

void
Blob::grow_right()
{
	while (buffer_[center_y_ * buffer_width_ + right_x_] == 255) {
		if (right_x_ < buffer_width_) {
			right_x_++;
		} else {
			return;
		}
		return;
	}
}

void
Blob::grow_up()
{
	while (buffer_[upper_y_ * buffer_width_ + center_x_] == 255) {
		if (upper_y_ > 0) {
			upper_y_--;
		} else {
			return;
		}
		return;
	}
}

int
Blob::getCenterX()
{
	center_x_ = left_x_ + (right_x_ - left_x_ / 2);
	return center_x_;
}

int
Blob::getCenterY()
{
	center_y_ = upper_y_ + (lower_y_ - upper_y_ / 2);
	return center_y_;
}

int
Blob::getLeftX() const
{
	return left_x_;
}

int
Blob::getLowerY() const
{
	return lower_y_;
}

int
Blob::getRightX() const
{
	return right_x_;
}

int
Blob::getUpperY() const
{
	return upper_y_;
}

void
Blob::setLeftX(int leftX)
{
	left_x_ = leftX;
}

void
Blob::setLowerY(int lowerY)
{
	lower_y_ = lowerY;
}

void
Blob::setRightX(int rightX)
{
	right_x_ = rightX;
}

void
Blob::setUpperY(int upperY)
{
	upper_y_ = upperY;
}

void
Blob::grow_down()
{
	while (buffer_[lower_y_ * buffer_width_ + center_x_] == 255) {
		if (lower_y_ < buffer_height_) {
			lower_y_++;
		} else {
			return;
		}
		return;
	}
}
