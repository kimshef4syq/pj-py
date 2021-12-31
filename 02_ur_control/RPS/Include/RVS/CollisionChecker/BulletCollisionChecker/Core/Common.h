// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <cstdio>
#include <cctype>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <boost/algorithm/string.hpp>
#include <RVS/CollisionChecker/BulletCollisionChecker/Core/Types.h>

namespace RVS
{
using ObjectPairKey = std::pair<std::string, std::string>;

/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
inline ObjectPairKey getObjectPairKey(const std::string &obj1,
                                      const std::string &obj2)
{
    return obj1 < obj2 ? std::make_pair(obj1, obj2)
                       : std::make_pair(obj2, obj1);
}


/**
 * @brief processResult Processes the ContactResult based on the information in
 * the ContactTestData
 * @param cdata Information used to process the results
 * @param contact Contacts from the collision checkers that will be processed
 * @param key Link pair used as a key to look up pair specific settings
 * @param found Specifies whether or not a collision has already been found
 * @return Pointer to the ContactResult.
 */
inline ContactResult *
processResult(ContactTestData &cdata, ContactResult &contact,
              const std::pair<std::string, std::string> &key, bool found)
{
    if (cdata.req.is_valid && !cdata.req.is_valid(contact)) return nullptr;

    if ((cdata.req.calculate_distance || cdata.req.calculate_penetration)
        && (contact.distance
            > cdata.collision_margin_data.getPairCollisionMargin(key.first,
                                                                 key.second)))
        return nullptr;

    if (!found) {
        ContactResultVector data;
        if (cdata.req.type == ContactTestType::FIRST) {
            data.emplace_back(contact);
            cdata.done = true;
        }
        else {
            data.reserve(100); // TODO: Need better way to initialize this
            data.emplace_back(contact);
        }

        return &(
            cdata.res->insert(std::make_pair(key, data)).first->second.back());
    }

    assert(cdata.req.type != ContactTestType::FIRST);
    ContactResultVector &dr = (*cdata.res)[key];
    if (cdata.req.type == ContactTestType::ALL) {
        dr.emplace_back(contact);
        return &(dr.back());
    }

    if (cdata.req.type == ContactTestType::CLOSEST) {
        if (contact.distance < dr[0].distance) {
            dr[0] = contact;
            return &(dr[0]);
        }
    }
    //    else if (cdata.cdata.condition == DistanceRequestType::LIMITED)
    //    {
    //      assert(dr.size() < cdata.req->max_contacts_per_body);
    //      dr.emplace_back(contact);
    //      return &(dr.back());
    //    }

    return nullptr;
}

/**
 * @brief This will check if a link is active provided a list. If the list is
 * empty the link is considered active.
 * @param active List of active link names
 * @param name The name of link to check if it is active.
 */
inline bool isLinkActive(const std::vector<std::string> &active,
                         const std::string &name)
{
    return active.empty()
           || (std::find(active.begin(), active.end(), name) != active.end());
}


/**
 * @brief Determine if contact is allowed between two objects.
 * @param name1 The name of the first object
 * @param name2 The name of the second object
 * @param acm The contact allowed function
 * @param verbose If true print debug informaton
 * @return True if contact is allowed between the two object, otherwise false.
 */
inline bool isContactAllowed(const std::string &name1, const std::string &name2,
                             const IsContactAllowedFn &acm,
                             bool verbose = false)
{
    // do not distance check geoms part of the same object / link / attached
    // body
    if (name1 == name2) return true;

    if (acm != nullptr && acm(name1, name2)) {
        if (verbose) {
            RVS_ERROR("Collision between {} and {} is "
                      "allowed. No contacts are computed.",
                      name1.c_str(), name2.c_str());
        }
        return true;
    }

    if (verbose) {
        RVS_ERROR("Actually checking collisions between {} and {}",
                  name1.c_str(), name2.c_str());
    }

    return false;
}

} // namespace RVS
