/*
 * BlobFarm.h
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

#ifndef BLOBFARM_H_
#define BLOBFARM_H_

#include "Blob.h"

#include <list>

class BlobFarm
{
public:
	BlobFarm(unsigned char *buffer,
	         int            buffer_width,
	         int            buffer_height,
	         int            min_diam,
	         int            max_diam,
	         int            merge_distance,
	         int            threshold);
	virtual ~BlobFarm();
	void              add_Blob(Blob *blob);
	void              add_Blob(int x, int y);
	std::list<Blob *> get_valid_blobs();

private:
	void           grow_blobs();
	void           merge_blobs();
	void           filter_blobs();
	Blob *         merge_blob(Blob *first, Blob *second);
	unsigned char *buffer_;

	int               buffer_width_;
	int               buffer_height_;
	int               min_diam_;
	int               max_diam_;
	int               merge_distance_;
	int               threshold_;
	std::list<Blob *> blobs;
};

#endif /* BLOBFARM_H_ */
