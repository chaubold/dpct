#include "userdata.h"
#include <stdexcept>

namespace dpct
{

NameData::NameData(const std::string& name):
	name_(name)
{}

std::ostream& operator<<(std::ostream& lhs, const NameData& rhs)
{
	lhs << rhs.getName();
	return lhs;
}

} // namespace dpct
