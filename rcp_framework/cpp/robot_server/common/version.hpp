// version.hpp
#pragma once

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 1

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define VERSION_STRING                                                         \
  TOSTRING(VERSION_MAJOR)                                                      \
  "." TOSTRING(VERSION_MINOR) "." TOSTRING(VERSION_PATCH)

const char *getServerVersion() {
  return VERSION_STRING;
}