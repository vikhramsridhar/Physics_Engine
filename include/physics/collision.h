#pragma once
#include "contact.h"
#include <vector>

// Each function returns true and fills `out` if the two bodies overlap.
// Dispatch by shape type happens in generateContact(), which any pair of
// bodies (regardless of shape) should be run through.

bool sphereSphereContact(RigidBody& a, RigidBody& b, Contact& out);
bool sphereBoxContact(RigidBody& sphereBody, RigidBody& boxBody, Contact& out);
bool boxBoxContact(RigidBody& a, RigidBody& b, std::vector<Contact>& out);      // SAT (face+edge) + manifold clipping, up to 4 points
bool spherePlaneContact(RigidBody& sphereBody, RigidBody& groundBody, float groundY, Contact& out);
bool boxPlaneContact(RigidBody& boxBody, RigidBody& groundBody, float groundY, std::vector<Contact>& out);

// Dispatches to the correct narrow-phase test based on shape.type of each body.
// groundY/groundBody are used only for the implicit ground plane at y = groundY.
void generateContacts(std::vector<RigidBody>& bodies, RigidBody& groundBody,
                       float groundY, std::vector<Contact>& contactsOut);
