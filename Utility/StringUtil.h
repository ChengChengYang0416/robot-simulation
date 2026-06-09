#pragma once

#include <string>
#include <string_view>

namespace Utility {

[[nodiscard]] std::string wideToUtf8( const wchar_t* wide );
// Converts a null-terminated wide-character string to UTF-8 using the Win32
// WideCharToMultiByte API. Returns an empty string when wide is null or empty.

[[nodiscard]] std::string wideToUtf8( std::wstring_view wide );
// Overload accepting a string_view; safe for non-null-terminated input.

}  // namespace Utility
