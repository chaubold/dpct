#ifndef DPCT_USERDATA_H
#define DPCT_USERDATA_H

#include <iostream>

namespace dpct
{

class UserData
{
};

class NameData : public UserData
{
public:
	NameData(const std::string& name);

	const std::string getName() const { return name_; }

private:
	std::string name_;
};

std::ostream& operator<<(std::ostream& lhs, const NameData& rhs);

} // namespace dpct


#endif // DPCT_NODE_H
