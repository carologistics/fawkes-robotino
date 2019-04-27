/*
 * BlobFarm.cpp
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

#include "BlobFarm.h"

#include <cmath>
#include <cstdlib>

using namespace std;

BlobFarm::BlobFarm(unsigned char *buffer,
                   int            buffer_width,
                   int            buffer_height,
                   int            min_diam,
                   int            max_diam,
                   int            merge_distance,
                   int            threshold)
: buffer_(buffer),
  buffer_width_(buffer_width),
  buffer_height_(buffer_width),
  min_diam_(min_diam),
  max_diam_(max_diam),
  merge_distance_(merge_distance),
  threshold_(threshold)
{
	// TODO Auto-generated constructor stub
}

void
BlobFarm::add_Blob(Blob *blob)
{
	blobs.push_back(blob);
}

void
BlobFarm::add_Blob(int x, int y)
{
	Blob *blob = new Blob(buffer_, buffer_width_, buffer_height_, threshold_, x, y);
	blobs.push_back(blob);
}

void
BlobFarm::grow_blobs()
{
	for (list<Blob *>::iterator it = blobs.begin(); it != blobs.end(); ++it) {
		(*it)->grow();
	}
}

list<Blob *>
BlobFarm::get_valid_blobs()
{
	grow_blobs();
	merge_blobs();
	filter_blobs();
	list<Blob *> blobcopy(blobs);
	return blobcopy;
}

void
BlobFarm::filter_blobs()
{
	list<Blob *>::iterator it = blobs.begin();
	while (it != blobs.end()) {
		int diameter = (*it)->getDiameter();
		if (diameter < min_diam_ || diameter > max_diam_) {
			blobs.erase(it);
		} else {
			it++;
		}
	}
}

void
BlobFarm::merge_blobs()
{
	list<Blob *>::iterator it1, it2;
	for (it1 = blobs.begin(); it1 != blobs.end(); ++it1) {
		it2 = it1;
		it2++; // we don't want to compare it with itsself
		while (it2 != blobs.end()) {
			if (it1 != it2) {
				double first_radius = (*it1)->getDiameter() / 2.0;
				double sec_radius   = (*it2)->getDiameter() / 2.0;
				if ((abs((*it1)->getCenterX() - (*it2)->getCenterX())
				     < first_radius + sec_radius + merge_distance_)
				    && (abs((*it1)->getCenterY() - (*it2)->getCenterY())
				        < first_radius + sec_radius + merge_distance_)) {
					blobs.push_back(merge_blob(*it1, *it2));
					blobs.erase(it1);
					blobs.erase(it2);
					it2 = blobs.begin();
				} else {
					it2++;
				}
			}
		}
	}
}

Blob *
BlobFarm::merge_blob(Blob *first, Blob *second)
{
	int   x      = round(abs(first->getCenterX() - second->getCenterX()) / 2.0);
	int   y      = round(abs(first->getCenterY() - second->getCenterY()) / 2.0);
	Blob *merged = new Blob(buffer_, buffer_width_, buffer_height_, threshold_, x, y);
	merged->setLeftX(min(first->getLeftX(), second->getLeftX()));
	merged->setRightX(max(first->getRightX(), second->getRightX()));
	merged->setUpperY(min(first->getUpperY(), second->getUpperY()));
	merged->setLowerY(max(first->getLowerY(), second->getLowerY()));
	return merged;
}

BlobFarm::~BlobFarm()
{
	// TODO Auto-generated destructor stub
}
