/***************************************************************************
 *  image_loader.cpp - Loader for image
 *
 *  Created: Thu May 5 10:23:50 2019
 *  Copyright  2019 Morian Sonnet
 ****************************************************************************/

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

#include "image_loader.h"
#include <exception>
#include <memory>

#include <fvutils/color/conversions.h>

TF_Plugin_Image_Loader::TF_Plugin_Image_Loader(
    std::string name, fawkes::Logger *logger,
    firevision::colorspace_t expected_colorspace, unsigned int width,
    unsigned int height, bool normalize, double norm_mean, double norm_std,
    bool late_open, bool late_start)
    : TF_Plugin_Loader(name, logger), should_colorspace_(expected_colorspace),
      width_(width), height_(height), own_final_buffer_(false),
      normalize_(normalize), normalize_mean_(norm_mean),
      normalize_std_(norm_std), late_open_(late_open),
      late_start_(late_start || late_open) {}

TF_Plugin_Image_Loader::~TF_Plugin_Image_Loader() {
  if (!late_start_)
    cam_->stop();
  if (!late_open_)
    cam_->close();
  delete cam_;
  if (own_final_buffer_)
    delete[] final_buffer_;
}

// TODO: check cam more detailed here
bool TF_Plugin_Image_Loader::verify() {
  try {
    cam_->open();
    cam_->start();
    if (late_start_)
      cam_->stop();
    if (late_open_)
      cam_->close();
  } catch (fawkes::Exception &e) {
    logger_->log_error(name_.c_str(), "Could not init source: %s", e.what());
    return false;
  }
  return true;
}

const void *TF_Plugin_Image_Loader::read() {
  if (late_open_)
    cam_->open();
  if (late_start_)
    cam_->start();
  cam_->capture();
  unsigned char *image_buffer = cam_->buffer();
  bool own_buffer = false;

  // resize is necessary if in and out image sizes do not coincide
  bool must_resize = (cam_->pixel_width() != width_ ||
                      cam_->pixel_height() != height_);

  // converting before resizing can per se only happen when must_resize is
  // activated Even then, it should only happen when the cam colorspace is not
  // usable for resizing
  bool must_preconvert =
      must_resize ? (colorspace_to_cv_type(cam_->colorspace()) < 0) : false;
  // The preconvert should convert in a  colorspace usable for resizing
  // If the should_colorspace is usable it is taken, otherwise RGB is used
  // For convenience in the postconvert, if no resize is done, this variable
  // still contains the colorspace at start of post_convert
  firevision::colorspace_t preconvert_to =
      must_preconvert ? (colorspace_to_cv_type(should_colorspace_) >= 0
                             ? should_colorspace_
                             : firevision::colorspace_t::RGB)
                      : cam_->colorspace();

  // Postconvert only needs to be executed when the colorspace after the
  // potential resize stage does not coincide with the output colorspace
  bool must_postconvert = preconvert_to != should_colorspace_;

  // convert the image into the right colorspace
  if (must_preconvert) {
    unsigned char *old_image_buffer = image_buffer;
    unsigned char *new_image_buffer = new unsigned char[colorspace_buffer_size(
        preconvert_to, width_, height_)];
    try {
      convert(old_image_buffer, new_image_buffer, cam_->colorspace(),
              preconvert_to, width_, height_);
    } catch (std::exception &e) {
      if (own_buffer)
        delete[] old_image_buffer;
      delete[] new_image_buffer;
    }
    if (own_buffer)
      delete[] old_image_buffer;
    image_buffer = new_image_buffer;
    own_buffer = true;
  }

  // sample down to should be image size
  //
  if (must_resize) {
    unsigned char *old_image_buffer = image_buffer;
    unsigned char *new_image_buffer = new unsigned char[colorspace_buffer_size(
        preconvert_to, width_, height_)];
    try {
      resize(old_image_buffer, new_image_buffer, preconvert_to,
             cam_->pixel_width(), cam_->pixel_height(), width_,
             height_);
    } catch (std::exception &e) {
      if (own_buffer)
        delete[] old_image_buffer;
      delete[] new_image_buffer;
    }
    if (own_buffer)
      delete[] old_image_buffer;
    image_buffer = new_image_buffer;
    own_buffer = true;
  }

  // convert the image into the right colorspace
  if (must_postconvert) {
    unsigned char *old_image_buffer = image_buffer;
    unsigned char *new_image_buffer = new unsigned char[colorspace_buffer_size(
        should_colorspace_, width_, height_)];
    try {
      convert(old_image_buffer, new_image_buffer, preconvert_to,
              should_colorspace_, width_, height_);
    } catch (std::exception &e) {
      if (own_buffer)
        delete[] old_image_buffer;
      delete[] new_image_buffer;
    }
    if (own_buffer)
      delete[] old_image_buffer;
    image_buffer = new_image_buffer;
    own_buffer = true;
  }

  if (normalize_) {
    try {
      switch (colorspace_to_base_type(should_colorspace_)) {
      default:
      case TYPE_UNSUPPORTED:
        logger_->log_error(name_.c_str(), "Unsupported type for normalizing");
        break;
      case TYPE_UINT:
        normalize(reinterpret_cast<unsigned int *>(image_buffer),
                  colorspace_buffer_size(should_colorspace_, width_, height_));
        break;
      case TYPE_INT:
        normalize(reinterpret_cast<int *>(image_buffer),
                  colorspace_buffer_size(should_colorspace_, width_, height_));
        break;
      case TYPE_FLOAT:
        normalize(reinterpret_cast<float *>(image_buffer),
                  colorspace_buffer_size(should_colorspace_, width_, height_));
        break;
      case TYPE_DOUBLE:
        normalize(reinterpret_cast<double *>(image_buffer),
                  colorspace_buffer_size(should_colorspace_, width_, height_));
        break;
      case TYPE_UCHAR:
        normalize(reinterpret_cast<unsigned char *>(image_buffer),
                  colorspace_buffer_size(should_colorspace_, width_, height_));
        break;
      case TYPE_CHAR:
        normalize(reinterpret_cast<char *>(image_buffer),
                  colorspace_buffer_size(should_colorspace_, width_, height_));
        break;
      case TYPE_CHAR16:
        normalize(reinterpret_cast<char16_t *>(image_buffer),
                  colorspace_buffer_size(should_colorspace_, width_, height_));
        break;
      }
    } catch (std::exception &e) {
      if (own_buffer)
        delete[] image_buffer;
    }
  }

  if (own_final_buffer_)
    delete[] final_buffer_; // do this just in case the user of this class did
                            // not call the post_read method
  own_final_buffer_ = true;
  final_buffer_ = image_buffer;


  return final_buffer_;
}

void TF_Plugin_Image_Loader::post_read() {
  if (late_start_)
    cam_->stop();
  if (late_open_)
    cam_->close();
  if (own_final_buffer_)
    delete[] final_buffer_;
  own_final_buffer_ = false; // make double delete not possible
}

void TF_Plugin_Image_Loader::resize(
    const unsigned char *in_buffer, unsigned char *out_buffer,
    firevision::colorspace_t colorspace, unsigned int old_width,
    unsigned int old_height, unsigned int new_width, unsigned int new_height) {
  int type = colorspace_to_cv_type(colorspace);                 // wonderful
  cv::Mat src(old_width, old_height, type, (void *)in_buffer);  // use AUTO_STEP
  cv::Mat dst(new_width, new_height, type, (void *)out_buffer); // use AUTO_STEP
  cv::resize(src, dst, dst.size(), /* compute scaling factors by dstsize */ 0,
             0, /*interpolation*/ cv::INTER_LINEAR);
}

void TF_Plugin_Image_Loader::convert(
    const unsigned char *in_buffer, unsigned char *out_buffer,
    firevision::colorspace_t old_colorspace,
    firevision::colorspace_t new_colorspace, unsigned int width,
    unsigned int height) {
  firevision::convert(old_colorspace, new_colorspace, in_buffer, out_buffer,
                      width, height);
}

int TF_Plugin_Image_Loader::colorspace_to_cv_type(
    firevision::colorspace_t colorspace) {
  switch (colorspace) {
  case firevision::colorspace_t::CS_UNKNOWN:
  case firevision::colorspace_t::YUV411_PLANAR:
  case firevision::colorspace_t::YUV411_PACKED:
  case firevision::colorspace_t::YUV420_PLANAR:
  case firevision::colorspace_t::YUY2:
  case firevision::colorspace_t::YVY2:
  case firevision::colorspace_t::YUV422_PACKED:
  case firevision::colorspace_t::YUV444_PACKED:
  case firevision::colorspace_t::YVU444_PACKED:
  case firevision::colorspace_t::YUV422_PLANAR:
  case firevision::colorspace_t::YUV422_PLANAR_QUARTER:
  case firevision::colorspace_t::BAYER_MOSAIC_RGGB:
  case firevision::colorspace_t::BAYER_MOSAIC_GBRG:
  case firevision::colorspace_t::BAYER_MOSAIC_GRBG:
  case firevision::colorspace_t::BAYER_MOSAIC_BGGR:
  case firevision::colorspace_t::RAW16:
  case firevision::colorspace_t::RGB_PLANAR:
  case firevision::colorspace_t::CARTESIAN_3D_FLOAT:
  case firevision::colorspace_t::CARTESIAN_3D_DOUBLE:
  case firevision::colorspace_t::CARTESIAN_3D_FLOAT_RGB:
  default:
    logger_->log_error(name_.c_str(),
                       "Should not use %s for anything here right now",
                       colorspace_to_string(colorspace));
    return -1;

  case firevision::colorspace_t::BGR:
  case firevision::colorspace_t::RGB:
    return CV_8UC3;

  case firevision::colorspace_t::MONO8:
  case firevision::colorspace_t::GRAY8:
    return CV_8UC1;

  case firevision::colorspace_t::RGB_WITH_ALPHA:
  case firevision::colorspace_t::BGR_WITH_ALPHA:
    return CV_8UC4;

  }
  return -1;
}

TF_Plugin_Image_Loader::BASE_TYPE
TF_Plugin_Image_Loader::colorspace_to_base_type(
    firevision::colorspace_t colorspace) {
  switch (colorspace) {
  case firevision::colorspace_t::CS_UNKNOWN:
  case firevision::colorspace_t::YUV411_PLANAR:
  case firevision::colorspace_t::YUV411_PACKED:
  case firevision::colorspace_t::YUV420_PLANAR:
  case firevision::colorspace_t::YUY2:
  case firevision::colorspace_t::YVY2:
  case firevision::colorspace_t::YUV422_PACKED:
  case firevision::colorspace_t::YUV444_PACKED:
  case firevision::colorspace_t::YVU444_PACKED:
  case firevision::colorspace_t::YUV422_PLANAR:
  case firevision::colorspace_t::YUV422_PLANAR_QUARTER:
  case firevision::colorspace_t::BAYER_MOSAIC_RGGB:
  case firevision::colorspace_t::BAYER_MOSAIC_GBRG:
  case firevision::colorspace_t::BAYER_MOSAIC_GRBG:
  case firevision::colorspace_t::BAYER_MOSAIC_BGGR:
  case firevision::colorspace_t::RAW16:
  case firevision::colorspace_t::RGB_PLANAR:
  case firevision::colorspace_t::CARTESIAN_3D_FLOAT:
  case firevision::colorspace_t::CARTESIAN_3D_DOUBLE:
  case firevision::colorspace_t::CARTESIAN_3D_FLOAT_RGB:
  default:
    logger_->log_error(name_.c_str(),
                       "Should not use %s for anything here right now",
                       colorspace_to_string(colorspace));
    return BASE_TYPE::TYPE_UNSUPPORTED;

  case firevision::colorspace_t::RGB_WITH_ALPHA:
  case firevision::colorspace_t::BGR_WITH_ALPHA:
  case firevision::colorspace_t::MONO8:
  case firevision::colorspace_t::GRAY8:
  case firevision::colorspace_t::BGR:
  case firevision::colorspace_t::RGB:
    return BASE_TYPE::TYPE_UCHAR;

  }
  return BASE_TYPE::TYPE_UNSUPPORTED;
}

template <typename T>
void TF_Plugin_Image_Loader::normalize(T *buffer, size_t size) {
  size /= sizeof(T);
  T mean = static_cast<T>(normalize_mean_);
  T std = static_cast<T>(normalize_std_);
  for (size_t i = 0; i < size; i++) {
    buffer[i] = ((buffer[i] - mean) / std);
  }
}
