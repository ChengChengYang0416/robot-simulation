#include "StringUtil.h"

#include <windows.h>

namespace Utility {

std::string wideToUtf8( const wchar_t* wide )
// Converts a null-terminated wide string to UTF-8 via WideCharToMultiByte.
// Returns "" when wide is null or empty.
{
	if( wide == nullptr || *wide == L'\0' ) {
		return {};
	}

	// First call: query required UTF-8 byte count (including null terminator).
	const int len = ::WideCharToMultiByte( CP_UTF8, 0, wide, -1, nullptr, 0, nullptr, nullptr );
	if( len <= 1 ) {
		return {};
	}

	std::string utf8( static_cast<size_t>( len - 1 ), '\0' );
	::WideCharToMultiByte( CP_UTF8, 0, wide, -1, &utf8[ 0 ], len, nullptr, nullptr );
	return utf8;
}

std::string wideToUtf8( std::wstring_view wide )
// Overload for non-null-terminated input: pass the explicit length so the API
// does not depend on a trailing L'\0'.
{
	if( wide.empty() ) {
		return {};
	}

	const int srcLen = static_cast<int>( wide.size() );
	const int len = ::WideCharToMultiByte( CP_UTF8, 0, wide.data(), srcLen, nullptr, 0, nullptr, nullptr );
	if( len <= 0 ) {
		return {};
	}

	std::string utf8( static_cast<size_t>( len ), '\0' );
	::WideCharToMultiByte( CP_UTF8, 0, wide.data(), srcLen, &utf8[ 0 ], len, nullptr, nullptr );
	return utf8;
}

}  // namespace Utility
