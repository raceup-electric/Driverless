#ifndef G2O_POSE_INFO_H
#define G2O_POSE_INFO_H

#include "g2o/types/data/g2o_types_data_api.h"
#include "graph_based_slam/NodeData.h"

#include <geometry_msgs/msg/transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/**
 * TODO:
 * \brief Represents info of a pose node
 */
class G2O_TYPES_DATA_API PoseData : public NodeData
{
public:
    PoseData();
    ~PoseData();

    virtual bool write(std::ostream &os) const;
    virtual bool read(std::istream &is);

    const int &getUniqueID() const { return unique_ID; }
    void setUniqueID(const int &id);

    const geometry_msgs::msg::Transform &getIdealCoords() const { return ideal_coords; }
    void setIdealCoords(const geometry_msgs::msg::Transform &ideal_data);

protected:
    // unique ID assigned to the cone
    int unique_ID;
    // save the ideal and ground truth coords of the perceived cone, in map frame
    geometry_msgs::msg::Transform ideal_coords;
};

#endif