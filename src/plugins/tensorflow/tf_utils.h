// Originally licensed under the MIT License
// <http://opensource.org/licenses/MIT>. Relicensed by Morian Sonnet 2019 to GPL
// license SPDX-License-Identifier: MIT Copyright (c) 2018 - 2019 Daniil
// Goncharov <neargye@gmail.com>.

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

#pragma once

#if defined(_MSC_VER)
#if !defined(COMPILER_MSVC)
#define COMPILER_MSVC // Set MSVC visibility of exported symbols in the shared
                      // library.
#endif
#pragma warning(push)
#pragma warning(disable : 4190)
#endif
#include <cstddef>
#include <cstdint>
#include <vector>

#include <tensorflow/c/c_api.h>

namespace tf_utils {

TF_Graph *LoadGraph(const char *graphPath);

void DeleteGraph(TF_Graph *graph);

TF_Session *CreateSession(TF_Graph *graph);

void DeleteSession(TF_Session *session);

TF_Code RunSession(TF_Session *session, const TF_Output *inputs,
                   TF_Tensor *const *input_tensors, std::size_t ninputs,
                   const TF_Output *outputs, TF_Tensor **output_tensors,
                   std::size_t noutputs);

TF_Code RunSession(TF_Session *session, const std::vector<TF_Output> &inputs,
                   const std::vector<TF_Tensor *> &input_tensors,
                   const std::vector<TF_Output> &outputs,
                   std::vector<TF_Tensor *> &output_tensors);

TF_Tensor *CreateTensor(TF_DataType data_type, const std::int64_t *dims,
                        std::size_t num_dims, const void *data,
                        std::size_t len);

template <typename T>
TF_Tensor *CreateTensor(TF_DataType data_type,
                        const std::vector<std::int64_t> &dims,
                        const std::vector<T> &data) {
  return CreateTensor(data_type, dims.data(), dims.size(), data.data(),
                      data.size() * sizeof(T));
}

TF_Tensor *CreateEmptyTensor(TF_DataType data_type, const std::int64_t *dims,
                             std::size_t num_dims);

TF_Tensor *CreateEmptyTensor(TF_DataType data_type,
                             const std::vector<std::int64_t> &dims);

void DeleteTensor(TF_Tensor *tensor);

void DeleteTensors(const std::vector<TF_Tensor *> &tensors);

void SetTensorsData(TF_Tensor *tensor, const void *data, std::size_t len);

template <typename T>
void SetTensorsData(TF_Tensor *tensor, const std::vector<T> &data) {
  SetTensorsData(tensor, data.data(), data.size() * sizeof(T));
}

template <typename T> std::vector<T> GetTensorsData(const TF_Tensor *tensor) {
  auto data = static_cast<T *>(TF_TensorData(tensor));
  if (data == nullptr) {
    return {};
  }

  return {data, data + (TF_TensorByteSize(tensor) /
                        TF_DataTypeSize(TF_TensorType(tensor)))};
}

template <typename T>
std::vector<std::vector<T>>
GetTensorsData(const std::vector<TF_Tensor *> &tensors) {
  std::vector<std::vector<T>> data;
  data.reserve(tensors.size());
  for (const auto t : tensors) {
    data.push_back(GetTensorsData<T>(t));
  }

  return data;
}

} // namespace tf_utils

#if defined(_MSC_VER)
#pragma warning(pop)
#endif
