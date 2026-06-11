#pragma once

#include <optional>
#include <string_view>
#include <TopoDS_Shape.hxx>

namespace Scene {

class StepLoader
{
public:
	[[nodiscard]] std::optional<TopoDS_Shape> read( std::wstring_view path );
	// Parses a STEP file at the given UTF-16 path and returns the merged root shape.
	// Returns std::nullopt on any failure (file not readable, no transferable roots,
	// or empty shape). No exceptions are thrown; callers handle the optional.
};

}  // namespace Scene
