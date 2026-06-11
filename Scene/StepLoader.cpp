#include "StepLoader.h"
#include "../Utility/StringUtil.h"
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>

namespace Scene {

std::optional<TopoDS_Shape> StepLoader::read( std::wstring_view path )
// STEPControl_Reader::ReadFile takes a UTF-8 path; convert via Utility::wideToUtf8
// (also handles non null-terminated string_view).
{
	const std::string utf8Path = Utility::wideToUtf8( path );

	STEPControl_Reader reader;
	const IFSelect_ReturnStatus status = reader.ReadFile( utf8Path.c_str() );
	if( status != IFSelect_RetDone ) {
		return std::nullopt;
	}

	const Standard_Integer roots = reader.TransferRoots();
	if( roots <= 0 ) {
		return std::nullopt;
	}

	TopoDS_Shape shape = reader.OneShape();
	if( shape.IsNull() ) {
		return std::nullopt;
	}

	return shape;
}

}  // namespace Scene
