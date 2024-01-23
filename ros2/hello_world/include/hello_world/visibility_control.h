// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HELLO_WORLD__VISIBILITY_CONTROL_H_
#define HELLO_WORLD__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HELLO_WORLD_EXPORT __attribute__ ((dllexport))
    #define HELLO_WORLD_IMPORT __attribute__ ((dllimport))
  #else
    #define HELLO_WORLD_EXPORT __declspec(dllexport)
    #define HELLO_WORLD_IMPORT __declspec(dllimport)
  #endif
  #ifdef HELLO_WORLD_BUILDING_DLL
    #define HELLO_WORLD_PUBLIC HELLO_WORLD_EXPORT
  #else
    #define HELLO_WORLD_PUBLIC HELLO_WORLD_IMPORT
  #endif
  #define HELLO_WORLD_PUBLIC_TYPE HELLO_WORLD_PUBLIC
  #define HELLO_WORLD_LOCAL
#else
  #define HELLO_WORLD_EXPORT __attribute__ ((visibility("default")))
  #define HELLO_WORLD_IMPORT
  #if __GNUC__ >= 4
    #define HELLO_WORLD_PUBLIC __attribute__ ((visibility("default")))
    #define HELLO_WORLD_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HELLO_WORLD_PUBLIC
    #define HELLO_WORLD_LOCAL
  #endif
  #define HELLO_WORLD_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // HELLO_WORLD__VISIBILITY_CONTROL_H_
