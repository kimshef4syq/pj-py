// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <vector>
#include <memory>
#include <map>
#include <array>
#include <unordered_map>
#include <functional>
#include <boost/bind.hpp>

#include <RVS/Environment/Geometry.h>
#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <RVS/CollisionChecker/BulletCollisionChecker/Core/CollisionMarginData.h>
RVS_COMMON_IGNORE_WARNINGS_POP

namespace RVS
{
using CollisionShapesConst = std::vector<std::shared_ptr<const Geometry>>;
using CollisionShapeConstPtr = std::shared_ptr<const Geometry>;
using CollisionShapePtr = std::shared_ptr<Geometry>;
using CollisionMarginData = CollisionMarginData;
using CollisionMarginOverrideType = CollisionMarginOverrideType;
using PairsCollisionMarginData = PairsCollisionMarginData;

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
using VectorIsometry3d = AlignedVector<Eigen::Isometry3d>;
/**
 * @brief Should return true if contact allowed, otherwise false.
 *
 * Also the order of strings should not matter, the function should handled by
 * the function.
 */
using IsContactAllowedFn =
    std::function<bool(const std::string &, const std::string &)>;

enum class ContinuousCollisionType
{
    CCType_None,
    CCType_Time0,
    CCType_Time1,
    CCType_Between
};

enum class ContactTestType
{
    FIRST = 0, /**< Return at first contact for any pair of objects */
    CLOSEST = 1, /**< Return the global minimum for a pair of objects */
    ALL = 2, /**< Return all contacts for a pair of objects */
    LIMITED = 3 /**< Return limited set of contacts for a pair of objects */
};

static const std::vector<std::string> ContactTestTypeStrings = {
    "FIRST",
    "CLOSEST",
    "ALL",
    "LIMITED",
};

struct ContactResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief The distance between two links */
    double distance{std::numeric_limits<double>::max()};
    /** @brief A user defined type id that is added to the contact shapes */
    std::array<int, 2> type_id{0, 0};
    /** @brief The two links that are in contact */
    std::array<std::string, 2> link_names;
    /** @brief The two shapes that are in contact. Each link can be made up of
     * multiple shapes */
    std::array<int, 2> shape_id{-1, -1};
    /** @brief Some shapes like octomap and mesh have subshape (boxes and
     * triangles) */
    std::array<int, 2> subshape_id{-1, -1};
    /** @brief The nearest point on both links in world coordinates */
    std::array<Eigen::Vector3d, 2> nearest_points{Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero()};
    /** @brief The nearest point on both links in local(link) coordinates */
    std::array<Eigen::Vector3d, 2> nearest_points_local{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    /** @brief The transform of link in world coordinates */
    std::array<Eigen::Isometry3d, 2> transform{Eigen::Isometry3d::Identity(),
                                               Eigen::Isometry3d::Identity()};
    /**
     * @brief The normal vector to move the two objects out of contact in world
     * coordinates
     *
     * @note This points from link_name[0] to link_name[1], so it shows the
     * direction to move link_name[1] to avoid or get out of collision with
     * link_name[0].
     */
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    /** @brief This is between 0 and 1 indicating the point of contact */
    std::array<double, 2> cc_time{-1, -1};
    /** @brief The type of continuous contact */
    std::array<ContinuousCollisionType, 2> cc_type{
        ContinuousCollisionType::CCType_None,
        ContinuousCollisionType::CCType_None};
    /** @brief The transform of link in world coordinates at its desired final
     * location. Note: This is not the location of the link at the point of
     * contact but the final location the link when performing continuous
     * collision checking. If you desire the location of contact use cc_time and
     * interpolate between transform and cc_transform;
     */
    std::array<Eigen::Isometry3d, 2> cc_transform{
        Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};

    /** @brief Some collision checkers only provide a single contact point for a
     * given pair. This is used to indicate if only one contact point is
     * provided which means nearest_points[0] must equal nearest_points[1].
     */
    bool single_contact_point{false};

    ContactResult() = default;

    /** @brief reset to default values */
    void clear()
    {
        distance = std::numeric_limits<double>::max();
        nearest_points[0].setZero();
        nearest_points[1].setZero();
        nearest_points_local[0].setZero();
        nearest_points_local[1].setZero();
        transform[0] = Eigen::Isometry3d::Identity();
        transform[1] = Eigen::Isometry3d::Identity();
        link_names[0] = "";
        link_names[1] = "";
        shape_id[0] = -1;
        shape_id[1] = -1;
        subshape_id[0] = -1;
        subshape_id[1] = -1;
        type_id[0] = 0;
        type_id[1] = 0;
        normal.setZero();
        cc_time[0] = -1;
        cc_time[1] = -1;
        cc_type[0] = ContinuousCollisionType::CCType_None;
        cc_type[1] = ContinuousCollisionType::CCType_None;
        cc_transform[0] = Eigen::Isometry3d::Identity();
        cc_transform[1] = Eigen::Isometry3d::Identity();
        single_contact_point = false;
    }
};


using ContactResultVector = AlignedVector<ContactResult>;

template <typename Key, typename Value>
using AlignedMap =
    std::map<Key, Value, std::less<Key>,
             Eigen::aligned_allocator<std::pair<const Key, Value>>>;
using ContactResultMap =
    AlignedMap<std::pair<std::string, std::string>, ContactResultVector>;

/**
 * @brief Should return true if contact results are valid, otherwise false.
 *
 * This is used so users may provide a callback to reject/approve collision
 * results in various algorithms.
 */
using IsContactResultValidFn = std::function<bool(const ContactResult &)>;

/** @brief The ContactRequest struct */
struct ContactRequest
{
    /** @brief This controls the exit condition for the contact test type */
    ContactTestType type = ContactTestType::ALL;

    /** @brief This enables the calculation of penetration contact data if two
     * objects are in collision */
    bool calculate_penetration = true;

    /** @brief This enables the calculation of distance data if two objects are
     * within the contact threshold */
    bool calculate_distance = true;

    /** @brief This is used if the ContactTestType is set to LIMITED, where the
     * test will exit when number of contacts reach this limit */
    long contact_limit = 0;

    /** @brief This provides a user defined function approve/reject contact
     * results */
    IsContactResultValidFn is_valid = nullptr;

    ContactRequest(ContactTestType type = ContactTestType::ALL) : type(type) {}
};

inline std::size_t flattenMoveResults(ContactResultMap &&m,
                                      ContactResultVector &v)
{
    v.clear();
    v.reserve(m.size());
    for (const auto &mv : m)
        std::move(mv.second.begin(), mv.second.end(), std::back_inserter(v));

    return v.size();
}

inline std::size_t flattenCopyResults(const ContactResultMap &m,
                                      ContactResultVector &v)
{
    v.clear();
    v.reserve(m.size());
    for (const auto &mv : m)
        std::copy(mv.second.begin(), mv.second.end(), std::back_inserter(v));

    return v.size();
}

// Need to mark deprecated
inline std::size_t flattenResults(ContactResultMap &&m, ContactResultVector &v)
{
    return flattenMoveResults(std::move(m), v);
}

#ifndef SWIG
/**
 * @brief This data is intended only to be used internal to the collision
 * checkers as a container and should not be externally used by other libraries
 * or packages.
 */
struct ContactTestData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactTestData() = default;
    ContactTestData(const std::vector<std::string> &active,
                    CollisionMarginData collision_margin_data,
                    IsContactAllowedFn fn, ContactRequest req,
                    ContactResultMap &res)
        : active(&active),
          collision_margin_data(std::move(collision_margin_data)),
          fn(std::move(fn)), req(std::move(req)), res(&res)
    {
    }

    /** @brief A vector of active links */
    const std::vector<std::string> *active = nullptr;

    /** @brief The current contact_distance threshold */
    CollisionMarginData collision_margin_data{0};

    /** @brief The allowed collision function used to check if two links should
     * be excluded from collision checking */
    IsContactAllowedFn fn = nullptr;

    /** @brief The type of contact request data */
    ContactRequest req;

    /** @brief Destance query results information */
    ContactResultMap *res = nullptr;

    /** @brief Indicate if search is finished */
    bool done = false;
};
#endif // SWIG

/**
 * @brief High level descriptor used in planners and utilities to specify what
 * kind of collision check is desired.
 *
 * DISCRETE - Discrete contact manager using only steps specified
 * LVS_DISCRETE - Discrete contact manager interpolating using longest valid
 * segment CONTINUOUS - Continuous contact manager using only steps specified
 * LVS_CONTINUOUS - Continuous contact manager interpolating using longest valid
 * segment
 */
enum class CollisionEvaluatorType
{
    /** @brief None */
    NONE,
    /** @brief Discrete contact manager using only steps specified */
    DISCRETE,
    /** @brief Discrete contact manager interpolating using longest valid
       segment */
    LVS_DISCRETE,
    /** @brief Continuous contact manager using only steps specified */
    CONTINUOUS,
    /** @brief Continuous contact manager interpolating using longest valid
       segment */
    LVS_CONTINUOUS
};

/**
 * @brief This is a high level structure containing common information that
 * collision checking utilities need. The goal of this config is to allow all
 * collision checking utilities and planners to use the same datastructure
 */
struct CollisionCheckConfig
{
    CollisionCheckConfig(
        double default_margin = 0, ContactRequest request = ContactRequest(),
        CollisionEvaluatorType type = CollisionEvaluatorType::DISCRETE,
        double longest_valid_segment_length = 0.005)
        : collision_margin_data(default_margin),
          contact_request(std::move(request)), type(type),
          longest_valid_segment_length(longest_valid_segment_length)
    {
    }

    /** @brief Identify how the collision margin data should be applied to the
     * contact manager */
    CollisionMarginOverrideType collision_margin_override_type{
        CollisionMarginOverrideType::NONE};
    /** @brief Stores information about how the margins allowed between
     * collision objects*/
    CollisionMarginData collision_margin_data;
    /** @brief ContactRequest that will be used for this check. Default test
     * type: FIRST*/
    ContactRequest contact_request;
    /** @brief Specifies the type of collision check to be performed. Default:
     * DISCRETE */
    CollisionEvaluatorType type;
    /** @brief Longest valid segment to use if type supports lvs. Default:
     * 0.005*/
    double longest_valid_segment_length{0.005};
};
} // namespace RVS
