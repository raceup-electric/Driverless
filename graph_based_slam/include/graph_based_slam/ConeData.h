#ifndef G2O_CONE_INFO_H
#define G2O_CONE_INFO_H

#include "g2o/types/data/g2o_types_data_api.h"
#include "graph_based_slam/NodeData.h"

#include <geometry_msgs/msg/point_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/**
 * \brief Represents color and unique ID of a landmark (cone)
 */
class G2O_TYPES_DATA_API ConeData : public NodeData
{
public:
    ConeData();
    ~ConeData();

    virtual bool write(std::ostream &os) const;
    virtual bool read(std::istream &is);

    const std::string &getColor() const { return cone_color; }
    void setColor(const std::string &color);

    const int &getUniqueID() const { return unique_ID; }
    void setUniqueID(const int &id);

    const geometry_msgs::msg::PointStamped &getIdealCoords() const { return ideal_coords; }
    void setIdealCoords(const geometry_msgs::msg::PointStamped &ideal_data);

protected:
    // color of the cone
    std::string cone_color;
    // unique ID assigned to the cone
    int unique_ID;
    // save the ideal and ground truth coords of the perceived cone, in map frame
    geometry_msgs::msg::PointStamped ideal_coords;
};

#endif