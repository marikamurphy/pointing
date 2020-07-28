#ifndef TF_BROADCAST_PR_H
#define TF_BROADCAST_PR_H

#include <string>
#include "PoseRecipient.h"

class TFBroadcastPR: public PoseRecipient {
public:
    TFBroadcastPR(std::string src, std::string dest);
    void receivePose(geometry_msgs::Pose &pose);
private:
    std::string _src;
    std::string _dest;
};

#endif
