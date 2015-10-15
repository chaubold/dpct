#include "iarcnotifier.h"
#include "arc.h"

namespace dpct
{

void IArcNotifier::registerObserverArc(Arc* arc)
{
    observerArcs_.push_back(arc);
}

void IArcNotifier::visitObserverArcs(NotificationFunction notificationFunc)
{
    for(std::vector<Arc*>::iterator it = observerArcs_.begin(); it != observerArcs_.end(); ++it)
    {
        notificationFunc(*it);
    }
}

} // namespace dpct
