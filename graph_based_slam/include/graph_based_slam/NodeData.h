#ifndef G2O_NODE_DATA_H
#define G2O_NODE_DATA_H

#include <iosfwd>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "g2o/core/optimizable_graph.h"
#include "g2o/types/data/g2o_types_data_api.h"

/**
 * @brief Additional data useful to identify and process graph nodes.
 */
class G2O_TYPES_DATA_API NodeData : public g2o::HyperGraph::Data
{
public:
    NodeData();
    virtual ~NodeData();

    rclcpp::Time getTimestamp() const { return timestamp; }
    void setTimestamp(rclcpp::Time ts);

    const std::string &getNodeType() const { return node_type; }
    void setNodeType(const std::string &n_type);

protected:
    // timestamp when the measurement was generated
    rclcpp::Time timestamp;
    // to distinguish different types of nodes (pose or landmark)
    std::string node_type;
};

#endif