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

#ifndef INTRA_PROCESS_TALKER_LISTENER__VISIBILITY_CONTROL_H_
#define INTRA_PROCESS_TALKER_LISTENER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INTRA_PROCESS_TALKER_LISTENER_EXPORT __attribute__ ((dllexport))
    #define INTRA_PROCESS_TALKER_LISTENER_IMPORT __attribute__ ((dllimport))
  #else
    #define INTRA_PROCESS_TALKER_LISTENER_EXPORT __declspec(dllexport)
    #define INTRA_PROCESS_TALKER_LISTENER_IMPORT __declspec(dllimport)
  #endif
  #ifdef INTRA_PROCESS_TALKER_LISTENER_BUILDING_DLL
    #define INTRA_PROCESS_TALKER_LISTENER_PUBLIC INTRA_PROCESS_TALKER_LISTENER_EXPORT
  #else
    #define INTRA_PROCESS_TALKER_LISTENER_PUBLIC INTRA_PROCESS_TALKER_LISTENER_IMPORT
  #endif
  #define INTRA_PROCESS_TALKER_LISTENER_PUBLIC_TYPE INTRA_PROCESS_TALKER_LISTENER_PUBLIC
  #define INTRA_PROCESS_TALKER_LISTENER_LOCAL
#else
  #define INTRA_PROCESS_TALKER_LISTENER_EXPORT __attribute__ ((visibility("default")))
  #define INTRA_PROCESS_TALKER_LISTENER_IMPORT
  #if __GNUC__ >= 4
    #define INTRA_PROCESS_TALKER_LISTENER_PUBLIC __attribute__ ((visibility("default")))
    #define INTRA_PROCESS_TALKER_LISTENER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INTRA_PROCESS_TALKER_LISTENER_PUBLIC
    #define INTRA_PROCESS_TALKER_LISTENER_LOCAL
  #endif
  #define INTRA_PROCESS_TALKER_LISTENER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // INTRA_PROCESS_TALKER_LISTENER__VISIBILITY_CONTROL_H_
