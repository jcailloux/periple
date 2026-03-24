#pragma once

#include <cstdarg>
#include <cstdio>

namespace periple::detail {

inline void log(const char* fmt, ...) {
	std::fprintf(stderr, "periple: ");
	std::va_list args;
	va_start(args, fmt);
	std::vfprintf(stderr, fmt, args);
	va_end(args);
}

} // namespace periple::detail