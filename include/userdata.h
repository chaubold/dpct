#ifndef DPCT_USERDATA_H
#define DPCT_USERDATA_H

#include <iostream>
#include <memory>

namespace dpct
{

class UserData
{
public:
    virtual std::string toString() const = 0;
};

typedef std::shared_ptr<UserData> UserDataPtr;

class UserDataHolder
{
public:
    UserDataHolder(UserDataPtr data);
    UserDataPtr getUserData() const { return data_; }

private:
    // how to store reference to hypotheses graph?
    UserDataPtr data_;
};

class NameData : public UserData
{
public:
	NameData(const std::string& name);

    virtual std::string toString() const { return name_; }

private:
	std::string name_;
};

std::ostream& operator<<(std::ostream& lhs, const UserData& rhs);
std::ostream& operator<<(std::ostream& lhs, UserDataPtr rhs);

} // namespace dpct


#endif // DPCT_NODE_H
