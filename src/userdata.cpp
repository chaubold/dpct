#include "userdata.h"
#include <stdexcept>

namespace dpct
{

UserData::UserData()
{
	throw std::runtime_error("UserData should not be used directly! Create your own subclass!");
}

} // namespace dpct
