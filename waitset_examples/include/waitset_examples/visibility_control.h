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

#ifndef WAITSET_EXAMPLES__VISIBILITY_CONTROL_H_
#define WAITSET_EXAMPLES__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WAITSET_EXAMPLES_EXPORT __attribute__ ((dllexport))
    #define WAITSET_EXAMPLES_IMPORT __attribute__ ((dllimport))
  #else
    #define WAITSET_EXAMPLES_EXPORT __declspec(dllexport)
    #define WAITSET_EXAMPLES_IMPORT __declspec(dllimport)
  #endif
  #ifdef WAITSET_EXAMPLES_BUILDING_DLL
    #define WAITSET_EXAMPLES_PUBLIC WAITSET_EXAMPLES_EXPORT
  #else
    #define WAITSET_EXAMPLES_PUBLIC WAITSET_EXAMPLES_IMPORT
  #endif
  #define WAITSET_EXAMPLES_PUBLIC_TYPE WAITSET_EXAMPLES_PUBLIC
  #define WAITSET_EXAMPLES_LOCAL
#else
  #define WAITSET_EXAMPLES_EXPORT __attribute__ ((visibility("default")))
  #define WAITSET_EXAMPLES_IMPORT
  #if __GNUC__ >= 4
    #define WAITSET_EXAMPLES_PUBLIC __attribute__ ((visibility("default")))
    #define WAITSET_EXAMPLES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WAITSET_EXAMPLES_PUBLIC
    #define WAITSET_EXAMPLES_LOCAL
  #endif
  #define WAITSET_EXAMPLES_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // WAITSET_EXAMPLES__VISIBILITY_CONTROL_H_
