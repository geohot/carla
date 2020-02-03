// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Buffer.h"

#ifdef __APPLE__
#define MSGPACK_DISABLE_LEGACY_NIL
#endif

#include <rpc/msgpack.hpp>

namespace carla {

  class MsgPack {
  public:

    template <typename T>
    static Buffer Pack(const T &obj) {
      namespace mp = ::clmdep_msgpack;
      mp::sbuffer sbuf;
      mp::pack(sbuf, obj);
      return Buffer(reinterpret_cast<const unsigned char *>(sbuf.data()), static_cast<uint64_t>(sbuf.size()));
    }

    template <typename T>
    static T UnPack(const Buffer &buffer) {
      namespace mp = ::clmdep_msgpack;
      return mp::unpack(reinterpret_cast<const char *>(buffer.data()), buffer.size()).template as<T>();
    }

    template <typename T>
    static T UnPack(const unsigned char *data, size_t size) {
      namespace mp = ::clmdep_msgpack;
      return mp::unpack(reinterpret_cast<const char *>(data), size).template as<T>();
    }
  };

} // namespace carla
