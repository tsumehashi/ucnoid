/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_FILE_UTIL_H
#define UCNOID_UTIL_FILE_UTIL_H

#include <filesystem>
#include <string>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

extern UCNOID_EXPORT const char* DLL_PREFIX;
extern UCNOID_EXPORT const char* DLL_SUFFIX;
extern UCNOID_EXPORT const char* DLL_EXTENSION;
extern UCNOID_EXPORT const char* EXEC_SUFFIX;
extern UCNOID_EXPORT const char* EXEC_EXTENSION;
extern UCNOID_EXPORT const char* PATH_DELIMITER;

UCNOID_EXPORT std::filesystem::path getCompactPath(const std::filesystem::path& path);
UCNOID_EXPORT void makePathCompact(std::filesystem::path& io_path);


UCNOID_EXPORT int findSubDirectory(
    const std::filesystem::path& directory,
    const std::filesystem::path& path,
    std::filesystem::path& out_subdirectory);

UCNOID_EXPORT bool findRelativePath(
    const std::filesystem::path& from,
    const std::filesystem::path& to,
    std::filesystem::path& out_relativePath);

UCNOID_EXPORT std::string getExtension(const std::filesystem::path& path);

/*
   The following functions were originally defined to support both the version 2 and 3 of
   the boost.filesystem library. However, supporting the version 2 was stopped, and the use
   of these functions should be replaced with the original functions of the version 3.
*/
UCNOID_EXPORT std::string getGenericPathString(const std::filesystem::path& path);
UCNOID_EXPORT bool checkAbsolute(const std::filesystem::path& path);
UCNOID_EXPORT std::filesystem::path getAbsolutePath(const std::filesystem::path& path);
UCNOID_EXPORT std::string getAbsolutePathString(const std::filesystem::path& path);
UCNOID_EXPORT std::string getFilename(const std::filesystem::path& path);
UCNOID_EXPORT std::string getFilename(const std::string& pathString);
UCNOID_EXPORT std::string getBasename(const std::filesystem::path& path);
UCNOID_EXPORT std::string getPathString(const std::filesystem::path& path);
UCNOID_EXPORT std::string getNativePathString(const std::filesystem::path& path);

UCNOID_EXPORT std::string toActualPathName(const std::string& pathName);

}   // inline namespace ucnoid
}
    
#include "FileUtil.cpp.h"

#endif
