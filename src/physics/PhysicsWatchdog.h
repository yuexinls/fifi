#pragma once
#include "RigidBody.h"
#include "Collision/ContactManifold.h"
#include "core/DebugLog.h"
#include <vector>
#include <memory>
#include <sstream>
#include <cmath>
#include <iomanip>

struct BodyDiagnostic {
    int   index        = 0;
    float linearSpeed  = 0.0f;
    float angularSpeed = 0.0f;
    bool  speedWarning = false;
    bool  nanWarning   = false;
};

struct ContactDiagnostic {
    int   bodyAIndex      = -1;
    int   bodyBIndex      = -1;
    float penetration     = 0.0f;
    float normalLength    = 0.0f;
    int   pointCount      = 0;
    bool  badNormal       = false;
    bool  deepPenetration = false;
};

struct WatchdogReport {
    std::vector<BodyDiagnostic>    bodies;
    std::vector<ContactDiagnostic> contacts;

    int totalSpeedWarnings    = 0;
    int totalNaNWarnings      = 0;
    int totalBadNormals       = 0;
    int totalDeepPenetrations = 0;
    int totalContacts         = 0;
};

class PhysicsWatchdog {
public:
    WatchdogReport report; // updated every step

    void analyse(
        const std::vector<std::unique_ptr<RigidBody>>& bodies,
        const std::vector<ContactManifold>& contacts)
    {
        report = {};
        report.totalContacts = (int)contacts.size();

        // body checks
        for (int i = 0; i < (int)bodies.size(); i++) {
            auto& b = bodies[i];
            BodyDiagnostic d;
            d.index        = i;
            d.linearSpeed  = b->linearVelocity.length();
            d.angularSpeed = b->angularVelocity.length();

            // NaN check
            if (std::isnan(b->position.x)         ||
                std::isnan(b->linearVelocity.x)   ||
                std::isnan(b->angularVelocity.x))
            {
                d.nanWarning = true;
                report.totalNaNWarnings++;
                std::ostringstream ss;
                ss << "Body " << i << " has NaN in position/velocity";
                DWARN3("nan_body_" + std::to_string(i), ss.str(), 0.5);
            }

            if (d.linearSpeed > WARN_SPEED_LINEAR ||
                d.angularSpeed > WARN_SPEED_ANGULAR)
            {
                d.speedWarning = true;
                report.totalSpeedWarnings++;
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2);
                ss << "Body " << i
                   << " overspeeding - linSpd=" << d.linearSpeed
                   << " angSpd=" << d.angularSpeed;
                DWARN("speed_body_" + std::to_string(i), ss.str());
            }

            report.bodies.push_back(d);
        }

        // contact checks
        std::unordered_map<RigidBody*, int> bodyIndex;
        for (int i = 0; i < (int)bodies.size(); i++)
            bodyIndex[bodies[i].get()] = i;

        for (auto& c : contacts) {
            ContactDiagnostic cd;
            cd.pointCount   = (int)c.contacts.size();
            cd.penetration  = c.penetrationDepth;
            cd.normalLength = c.normal.length();
            cd.bodyAIndex   = bodyIndex.count(c.bodyA) ? bodyIndex[c.bodyA] : -1;
            cd.bodyBIndex   = bodyIndex.count(c.bodyB) ? bodyIndex[c.bodyB] : -1;

            // bad normal check (not close to unit length)
            float nLenSq = c.normal.lengthSq();
            if (nLenSq < 0.9f || nLenSq > 1.1f) {
                cd.badNormal = true;
                report.totalBadNormals++;
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(4);
                ss << "Contact [" << cd.bodyAIndex << "x" << cd.bodyBIndex
                   << "] bad normal length=" << cd.normalLength
                   << " n=(" << c.normal.x << ","
                              << c.normal.y << ","
                              << c.normal.z << ")";
                DWARN("bad_normal_" + std::to_string(cd.bodyAIndex)
                                    + "_" + std::to_string(cd.bodyBIndex),
                      ss.str());
            }

            // deep penetration check
            if (c.penetrationDepth > WARN_PENETRATION) {
                cd.deepPenetration = true;
                report.totalDeepPenetrations++;
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(4);
                ss << "Contact [" << cd.bodyAIndex << "x" << cd.bodyBIndex
                   << "] deep penetration=" << c.penetrationDepth;
                DWARN("deep_pen_" + std::to_string(cd.bodyAIndex)
                                  + "_" + std::to_string(cd.bodyBIndex),
                      ss.str());
            }

            // zero contact points check
            if (cd.pointCount == 0) {
                std::ostringstream ss;
                ss << "Contact [" << cd.bodyAIndex << "x" << cd.bodyBIndex
                   << "] has 0 contact points!!!!!!!";
                DWARN("zero_pts_" + std::to_string(cd.bodyAIndex)
                                  + "_" + std::to_string(cd.bodyBIndex),
                      ss.str());
            }

            report.contacts.push_back(cd);
        }
    }
};


