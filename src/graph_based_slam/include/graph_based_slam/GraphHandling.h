#ifndef GRAPH_HANDLING_H
#define GRAPH_HANDLING_H

// ROS2 headers
#include <rclcpp/rclcpp.hpp>

// g2o headers
#include <g2o/core/hyper_graph.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solver_eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/types_slam2d.h>

// Eigen headers
#include <Eigen/Dense>

/**
 * This class contains all the necessary methods to correctly handle the pose graph.
 * It uses g2o library to perform all the operations, included the optimization.
 */

class GraphHandling
{
public:
    // constructor
    // GraphHandling(const rclcpp::NodeOptions &options);
    GraphHandling(const std::string &solver_type = "lm_var");

    // destructor
    virtual ~GraphHandling();

    /**
     * @return The number of vertices of the graph.
     */
    int num_vertices() const;

    /**
     * @return The number of edges of the graph.
     */
    int num_edges() const;

    /**
     * @brief Create a SE2 vertex for the graph (x, y, theta coordinates). It represents a robot pose.
     * @param pose Matrix representing the specified pose.
     * @return The created node.
     */
    g2o::VertexSE2 *createPoseVertex(const Eigen::Isometry2d &pose);

    /**
     * @brief Actually add a SE2 vertex to the graph (x, y, theta coordinates). It represents a robot pose.
     * @param vertex A pointer to the vertex to be added.
     * @return The added node.
     */
    g2o::VertexSE2 *addPoseVertex(g2o::VertexSE2 *vertex);

    /**
     * @brief Create a a 2D point vertex for the graph (x, y coordinates). It represents a landmark position.
     * @param  xy_coords Vector representing the specified position.
     * @param color Additional information to store the estimated color of the cone.
     * @return The created node.
     */
    g2o::VertexPointXY *createLandmarkVertex(const Eigen::Vector2d &xy_coords);

    /**
     * @brief Actually add a 2D point vertex to the graph (x, y coordinates). It represents a landmark position.
     * @param vertex A pointer to the vertex to be added.
     * @return The added node.
     */
    g2o::VertexPointXY *addLandmarkVertex(g2o::VertexPointXY *vertex);

    /**
     * @brief Create an edge between two SE2 nodes of the graph. It represents a pose-pose edge.
     * @param node1 The node of origin of the edge.
     * @param node2 The node of destination of the edge.
     * @param rel_pose The relative pose between the two vertices.
     * @param info_matrix The associated information matrix.
     * @return The created edge.
     */
    g2o::EdgeSE2 *createPoseEdge(g2o::VertexSE2 *node1, g2o::VertexSE2 *node2, const Eigen::Isometry2d &rel_pose, const Eigen::MatrixXd &info_matrix);

    /**
     * @brief Actually add an edge between two SE2 nodes to the graph. It represents a pose-pose edge.
     * @param edge A pointer to the edge to be added.
     * @return The added edge.
     */
    g2o::EdgeSE2 *addPoseEdge(g2o::EdgeSE2 *edge);

    // TODO: finish specs here

    /**
     * @brief Create an edge between a SE2 and a 2D point node of the graph. It represents a pose-landmark edge.
     * @param pose_vertex The node representing a robot pose (origin of the edge).
     * @param land_vertex The node representing landmark position (destination of the edge).
     * @param xy_coords xy coordinates
     * @param info_matrix The associated information matrix.
     * @return The created edge.
     */
    g2o::EdgeSE2PointXY *createLandmarkEdge(g2o::VertexSE2 *pose_vertex, g2o::VertexPointXY *land_vertex, const Eigen::Vector2d &xy_coords, const Eigen::MatrixXd &info_matrix);

    /**
     * @brief Actually add an edge between a SE2 and a 2D point node of the graph. It represents a pose-landmark edge.
     * @param edge A pointer to the edge to be added.
     * @return The added edge.
     */
    g2o::EdgeSE2PointXY *addLandmarkEdge(g2o::EdgeSE2PointXY *edge);

    /**
     * @brief Globally optimize the given graph.
     * @param num_iterations Maximum number of optimization iterations.
     * @return The number of iterations performed by the optimizer.
     */
    int globalOptimization(int num_iterations);

    /**
     * @brief Locally optimize the given graph.
     * @param num_iterations Maximum number of optimization iterations.
     * @param edges_to_optim Subset of edges representing the portion of the graph to be optimized.
     * @return The number of iterations performed by the optimizer.
     */
    int localOptimization(int num_iterations, g2o::HyperGraph::EdgeSet edges_to_optim);

    /**
     * @brief Save the pose graph to a file.
     * @param filename The file in which to save the graph.
     */
    void saveGraph(const std::string &filename);

    /**
     * @brief Load a pose graph from file.
     * @param filename The file from which the graph will be loaded.
     * @return The result of the operation (success or not).
     */
    bool loadGraph(const std::string &filename);

    /**
     *@return A pointer to the optimizable graph.
     */
    g2o::SparseOptimizer *getGraph();

    /**
     *@return All the nodes in the graph.
     */
    g2o::HyperGraph::VertexIDMap getNodes();

    /**
     *@return All the edges in the graph.
     */
    g2o::HyperGraph::EdgeSet getEdges();

    /**
     * @return The last robot pose node added to the graph.
     */
    g2o::VertexSE2 *getLastPoseNode();

    /**
     * @return The last landmark node added to the graph.
     */
    g2o::VertexPointXY *getLastLandmarkNode();

private:
    // only one instance allowed
    std::unique_ptr<g2o::HyperGraph> slam_graph;
    g2o::VertexSE2 *last_added_pose;
    g2o::VertexPointXY *last_added_cone;
};

#endif // GRAPH_HANDLING_H