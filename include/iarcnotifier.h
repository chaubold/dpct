#ifndef DPCT_I_ARC_NOTIFIER_H
#define DPCT_I_ARC_NOTIFIER_H

#include <vector>
#include <functional>
#include <utility>

namespace dpct
{

class Arc;

class IArcNotifier
{
public:
	typedef std::function<void(Arc*)> NotificationFunction;

	void registerObserverArc(Arc* arc);

	void notifyObserverArcs(NotificationFunction notificationFunc);

protected:
	std::vector<Arc*> observerArcs_;
};

} // namespace dpct

#endif
